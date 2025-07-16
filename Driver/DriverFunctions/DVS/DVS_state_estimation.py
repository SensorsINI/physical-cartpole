# /*------------------------------------------------------*\
# | Telluride Neuromorphic Workshop: Cartpole TCP Output  |
# |  - Pure match filter detection (no Kalman, no y fix)  |
# |  - Outputs cart_x (always as detected), cart velocity |
# |    (raw diff), pole angle (from Hough), pole omega    |
# |  - Calibration phase: 'c' to start/end                |
# |  - Live TCP and console output                        |
# |  - Detected cart y is always true detected y          |
# |  - Easy parameter tuning at top                       |
# \*------------------------------------------------------*/

import dv_processing as dv
import cv2
import time
import numpy as np
import threading
from datetime import timedelta

from Driver.DriverFunctions.DVS.angle_pos_zmq import (
    start_zmq_server, stop_zmq_server, publish_estimate
)

# --- USER TUNED MATCH FILTER PARAMETERS ---
CART_RADIUS         = 5      # px, radius of template for match filter
CART_THRESH         = 0.50   # threshold for match quality (0-1)
TRACK_LENGTH_METERS = 0.396   # physical track length for calibration

# max linear speed: 1 m/s
MAX_LINEAR = 1.0
# max angular speed: 4 rotations/sec → 4 * 2π rad/s
MAX_ANGULAR = 4 * 2 * np.pi


# --- Global/Shared State ---
latest_detection = {
    "frame": None, "line": None,
    "cart_x": None, "cart_y": None,
    "angle": None, "timestamp": None
}
lock      = threading.Lock()
quit_flag = {'quit': False}

# previous for velocity computation
prev_cart_x      = None
prev_cart_x_time = None
prev_angle       = None
prev_angle_time  = None

prev_linear_velocity  = 0.0
prev_angular_velocity = 0.0

# fallback for angle
last_angle = None

# calibration state
calibrating      = False
calib_cart_xs    = []
calib_cart_ys    = []
fixed_pivot_y    = None
cart_min_x       = None
cart_max_x       = None
last_cart_x      = None
last_cart_y      = None
PIXELS_PER_METER = None
PIXEL_CENTER     = None


# ---- NEW: keep track of last *valid* kinematic state -------------
last_valid_cart_x      = None
last_valid_cart_time   = None
last_valid_angle       = None
last_valid_angle_time  = None
prev_linear_velocity_px_s = 0.0   # last valid linear speed in *pixels/s*



def make_circle_template(radius, thickness=-1, image_size=None):
    size = 2 * radius + 5 if image_size is None else image_size
    template = np.zeros((size, size), dtype=np.uint8)
    center = (size//2, size//2)
    cv2.circle(template, center, radius, 255, thickness)
    return template


CIRCLE_TEMPLATE = make_circle_template(CART_RADIUS)
TEMPLATE_H, TEMPLATE_W = CIRCLE_TEMPLATE.shape           # helper aliases
Y_TOLERANCE_PX = (TEMPLATE_H + 1) // 2
GATE_PX_STATIC   = 25          # max Δx from pole pivot (pixels)
ADAPTIVE_ALPHA   = 0.1         # EWMA for adapting template-score threshold
MIN_THRESH       = 0.35        # never let threshold drop below this
tmpl_threshold   = CART_THRESH   # start with the user value
MATCH_ROI_HALF = GATE_PX_STATIC + 5

def line_horizontal_intersect(x1, y1, x2, y2, y_horiz, img_width):
    if abs(y2 - y1) < 1e-3:
        return None
    t = (y_horiz - y1) / (y2 - y1)
    px = x1 + t * (x2 - x1)
    return px if 0.0 <= px < img_width else None

def gaussian_blur_uint8(img):
    return cv2.GaussianBlur(img, (3, 3), 0)

def detect_cart_robust(gray, last_x, last_y, pivot_x_pred, H, W):
    """
    Returns (cx, cy, score) if a gated detection passes all three guards,
    else None.
    """

    if cart_min_x is None or cart_max_x is None:
        return None

    if last_x is None:
        last_x = (cart_min_x + cart_max_x) // 2

    # ---- ROI guard -------------------------------------------------
    xmin = max(cart_min_x, int(last_x) - MATCH_ROI_HALF)
    xmax = min(cart_max_x, int(last_x) + MATCH_ROI_HALF)
    ymin = max(0,          fixed_pivot_y - Y_TOLERANCE_PX)
    ymax = min(H,          fixed_pivot_y + Y_TOLERANCE_PX)
    if xmin >= xmax or ymin >= ymax:
        return None

    roi = gray[ymin:ymax, xmin:xmax]
    roi_h, roi_w = roi.shape
    if roi_h < TEMPLATE_H or roi_w < TEMPLATE_W:
        # ROI too small for safe template matching → skip this slice
        return None
    blurred = gaussian_blur_uint8(roi)
    res = cv2.matchTemplate(blurred,
                            CIRCLE_TEMPLATE,
                            cv2.TM_CCOEFF_NORMED)
    _, score, _, loc = cv2.minMaxLoc(res)

    # ---- Template‐score guard -------------------------------------
    global tmpl_threshold
    # 1) First compare…
    if score < tmpl_threshold:
        return None

    # 2) Then update (EWMA) only on accepted detections:
    tmpl_threshold = max(
        MIN_THRESH,
        (1 - ADAPTIVE_ALPHA)*tmpl_threshold
        + ADAPTIVE_ALPHA*score
    )

    # Map ROI coords -> full-image coords
    cx = xmin + loc[0] + CIRCLE_TEMPLATE.shape[1]//2
    cy = ymin + loc[1] + CIRCLE_TEMPLATE.shape[0]//2

    cy = fixed_pivot_y

    # ---- Kinematic-consistency guard ------------------------------
    if abs(cx - pivot_x_pred) > GATE_PX_STATIC:
        return None

    return cx, cy, score


def process_events(events, visualizer):
    global calibrating, calib_cart_xs, calib_cart_ys, fixed_pivot_y
    global last_cart_x, last_cart_y
    global prev_cart_x, prev_cart_x_time, prev_angle, prev_angle_time
    global prev_angular_velocity, prev_linear_velocity
    global last_angle
    global last_valid_cart_x, last_valid_cart_time, last_valid_angle, last_valid_angle_time, prev_linear_velocity_px_s

    # preprocess
    frame = visualizer.generateImage(events)
    gray  = (cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
             if frame.ndim == 3 else frame)
    H, W  = gray.shape

    # ── CALIBRATION PHASE ──────────────────────────────────────────
    if calibrating or fixed_pivot_y is None:
        template = CIRCLE_TEMPLATE
        blurred  = gaussian_blur_uint8(gray)
        result   = cv2.matchTemplate(blurred, template,
                                     cv2.TM_CCOEFF_NORMED)
        _, conf, _, loc = cv2.minMaxLoc(result)
        if conf >= CART_THRESH:
            cx = loc[0] + template.shape[1] // 2
            cy = loc[1] + template.shape[0] // 2
            last_cart_x, last_cart_y = cx, cy
            if calibrating:
                calib_cart_xs.append(cx)
                calib_cart_ys.append(cy)
        else:
            cx = last_cart_x if last_cart_x is not None else W // 2
            cy = last_cart_y if last_cart_y is not None else H // 2

        return {
            "frame":    frame,
            "line":     None,
            "cart_x":   cx,
            "cart_y":   cy,
            "angle":    None,
            "timestamp": time.time()
        }

    # ── RUNTIME: Hough + INTERSECTION FILTERING ────────────────────
    _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY)
    edges     = cv2.Canny(binary, 50, 150, apertureSize=3)
    lines     = cv2.HoughLinesP(edges, 1, np.pi/180,
                                threshold=40,
                                minLineLength=20,
                                maxLineGap=10)

    line      = None
    angle_rad = None
    pivot_y   = fixed_pivot_y

    if lines is not None:
        best_len = 0
        best_seg = None
        for L in lines:
            x1, y1, x2, y2 = map(float, L[0])
            px = line_horizontal_intersect(x1, y1, x2, y2, pivot_y, W)
            if px is None:
                continue
            if px < cart_min_x or px > cart_max_x:
                continue
            length = np.hypot(x2 - x1, y2 - y1)
            if length > best_len:
                best_len = length
                best_seg = (x1, y1, x2, y2, px)

        if best_seg is not None:
            x1, y1, x2, y2, px = best_seg
            pivot_x = px
            # pick tip
            if np.hypot(x1-px, y1-pivot_y) > np.hypot(x2-px, y2-pivot_y):
                tip_x, tip_y = x1, y1
            else:
                tip_x, tip_y = x2, y2
            line      = (pivot_x, pivot_y, tip_x, tip_y)
            dx, dy    = tip_x-pivot_x, tip_y-pivot_y
            angle_rad = np.arctan2(-dx, -dy)

    # fallback to last_angle or zero
    if angle_rad is not None:
        last_angle = angle_rad
    else:
        angle_rad = last_angle if last_angle is not None else 0.0

    # decide cart_x
    # decide cart_x: prefer Hough pivot, else last known position
    pivot_x_pred = pivot_x if line is not None else last_cart_x

    # ── EDGE CASE: if still None (first slice), revert to image centre ──
    if pivot_x_pred is None:
        pivot_x_pred = PIXEL_CENTER

    cart_det = detect_cart_robust(gray,
                                  last_cart_x,
                                  last_cart_y,
                                  pivot_x_pred,
                                  H, W)

    if cart_det is not None:
        # High-confidence, gated template match
        cx, cy, _ = cart_det
    else:
        # Fallback: trust pole geometry or last valid
        cx, cy = pivot_x_pred, pivot_y


    # ─── VELOCITIES & TIMESTAMP ───────────────────────────────────
    # 1) obtain a consistent timestamp for this batch:
    #    * If your EventStore exposes timestamps, use that (psuedocode here):
    #       ts = events.getLastEventTimestamp()  # in seconds
    #    * Otherwise, fall back to wall‑clock:
    ts = time.time()

    # ─── HARD‑LIMIT GATE for position & angle ─────────────────────────
    if PIXELS_PER_METER is not None:  # only after calibration
        # ---------- linear position -----------------------------------
        valid_cart = True  # assume OK until proven impossible
        if last_valid_cart_x is not None:
            dt_clip = ts - last_valid_cart_time
            if dt_clip > 0:
                max_dx_px = dt_clip * MAX_LINEAR * PIXELS_PER_METER
                x_pred = last_valid_cart_x + prev_linear_velocity_px_s * dt_clip
                if abs(cx - x_pred) > max_dx_px:  # impossible ⇒ reject
                    cx = int(round(x_pred))  # substitute prediction
                    valid_cart = False

        if valid_cart:  # update *only* on success
            last_valid_cart_x = cx
            last_valid_cart_time = ts

        # ---------- angular position ----------------------------------
        valid_ang = True
        if last_valid_angle is not None:
            dt_clip = ts - last_valid_angle_time
            if dt_clip > 0:
                max_dtheta = dt_clip * MAX_ANGULAR
                theta_pred = (last_valid_angle + prev_angular_velocity * dt_clip + np.pi) % (2 * np.pi) - np.pi
                dtheta = angle_rad - theta_pred
                if dtheta > np.pi: dtheta -= 2 * np.pi
                if dtheta < -np.pi: dtheta += 2 * np.pi
                if abs(dtheta) > max_dtheta:  # impossible ⇒ reject
                    angle_rad = theta_pred
                    valid_ang = False
                else:
                    valid_ang = True


        if valid_ang:
            last_valid_angle = angle_rad
            last_valid_angle_time = ts

    last_cart_x, last_cart_y = cx, cy

    # 2) LINEAR velocity (px/s → m/s):
    #    Δx is in pixels; Δt may vary, so we divide by ts_prev.
    linear_velocity_px_s = 0.0
    if prev_cart_x is not None and prev_cart_x_time is not None:
        dt = ts - prev_cart_x_time
        if dt > 0:
            linear_velocity_px_s = (cx - prev_cart_x) / dt
    # update previous for next slice
    prev_cart_x, prev_cart_x_time = cx, ts

    # convert to meters per second if calibrated
    if PIXELS_PER_METER is not None:
        linear_velocity = linear_velocity_px_s / PIXELS_PER_METER
    else:
        linear_velocity = linear_velocity_px_s  # fallback

    # 3) ANGULAR velocity (rad/s):
    angular_velocity_rad_s = 0.0
    if prev_angle is not None and prev_angle_time is not None:
        dt = ts - prev_angle_time
        if dt > 0:
            # compute smallest angular difference to handle wrap‑around at ±π
            delta_rad = angle_rad - prev_angle
            if delta_rad > np.pi:
                delta_rad -= 2 * np.pi
            elif delta_rad < -np.pi:
                delta_rad += 2 * np.pi
            angular_velocity_rad_s = delta_rad / dt
    # update previous for next slice
    prev_angle, prev_angle_time = angle_rad, ts

    # ─── VELOCITY CAPPING ────────────────────────────────────────

    # cap linear velocity
    if abs(linear_velocity) > MAX_LINEAR:
        # spike detected → use last valid
        linear_velocity = prev_linear_velocity
    else:
        prev_linear_velocity = linear_velocity

    # cap angular velocity
    if abs(angular_velocity_rad_s) > MAX_ANGULAR:
        # spike detected → use last valid
        angular_velocity_rad_s = prev_angular_velocity
    else:
        prev_angular_velocity = angular_velocity_rad_s

    # keep a pixel/s copy for the next prediction corridor
    prev_linear_velocity_px_s = (
        linear_velocity * PIXELS_PER_METER
        if PIXELS_PER_METER
        else linear_velocity_px_s
    )

    # 4) now you can publish using your usual API:
    if PIXELS_PER_METER is not None and PIXEL_CENTER is not None:
        publish_estimate(
            angle_rad        = angle_rad,
            cart_x           = (cx - PIXEL_CENTER) / PIXELS_PER_METER,
            linear_velocity  = linear_velocity,
            angular_velocity = angular_velocity_rad_s,
            timestamp        = ts,
        )
        print(f"[DATA] t={ts:.3f} cart_x={cx:.0f} "
              f"v={linear_velocity:.2f} m/s "
              f"angle={np.rad2deg(angle_rad):.1f}° "
              f"omega={angular_velocity_rad_s:.2f} rad/s")

    # ─── RETURN THE DICT ──────────────────────────────────────────
    return {
        "frame":     frame,
        "line":      line,
        "cart_x":    cx,
        "cart_y":    pivot_y,
        "angle":     angle_rad,
        "timestamp": ts
    }

def visualisation_thread():
    global calibrating, calib_cart_xs, calib_cart_ys, fixed_pivot_y
    global cart_min_x, cart_max_x, PIXELS_PER_METER, PIXEL_CENTER

    cv2.namedWindow("Preview", cv2.WINDOW_NORMAL)
    waiting_overlay = True

    while not quit_flag['quit']:
        with lock:
            frame  = latest_detection["frame"]
            line   = latest_detection["line"]
            cart_x = latest_detection["cart_x"]
            cart_y = latest_detection["cart_y"]

        if frame is not None:
            waiting_overlay = False
            vis = frame.copy()

            # ── draw pivot line ────────────────────────────────
            if fixed_pivot_y is not None:
                # full‑width horizontal line at the calibrated pivot y
                cv2.line(vis,
                         (0, fixed_pivot_y),
                         (vis.shape[1], fixed_pivot_y),
                         (255, 0, 255), 1)  # magenta, 1px thick

            # ── draw left/right boundaries ───────────────────────
            if cart_min_x is not None and cart_max_x is not None:
                # left boundary
                cv2.line(vis,
                         (cart_min_x, 0),
                         (cart_min_x, vis.shape[0]),
                         (255, 255, 0), 1)  # cyan, 1px thick
                # right boundary
                cv2.line(vis,
                         (cart_max_x, 0),
                         (cart_max_x, vis.shape[0]),
                         (255, 255, 0), 1)

            # ── existing overlays ────────────────────────────────
            # Red pole
            if line is not None:
                x1, y1, x2, y2 = line
                cv2.line(vis, (int(x1), int(y1)),
                         (int(x2), int(y2)),
                         (0, 0, 255), 2)
            # Green cart (always at detected y)
            if cart_x is not None and cart_y is not None:
                cv2.circle(vis,
                           (int(cart_x), int(cart_y)),
                           CART_RADIUS,
                           (0, 255, 0), 2)

            if calibrating:
                cv2.putText(vis,
                            "CALIBRATING: move cart to ends, press 'c'",
                            (30, 40),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.7, (0,0,255), 2)

            cv2.imshow("Preview", vis)
        else:
            # … (unchanged waiting overlay) …
            if waiting_overlay:
                black = np.zeros((240, 346, 3), dtype=np.uint8)
                cv2.putText(black, "Waiting for events...",
                            (60, 120),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (255,255,255), 2)
                cv2.imshow("Preview", black)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            quit_flag['quit'] = True
        elif key == ord('c'):
            # … (unchanged calibration logic) …
            if not calibrating:
                print("[CALIBRATION] Starting: move cart to both ends, then press 'c' again.")
                calibrating = True
                calib_cart_xs.clear()
                calib_cart_ys.clear()
            else:
                calibrating = False
                if calib_cart_xs:
                    fixed_pivot_y    = int(np.median(calib_cart_ys))
                    cart_min_x       = int(np.min(calib_cart_xs))
                    cart_max_x       = int(np.max(calib_cart_xs))
                    PIXELS_PER_METER = (cart_max_x - cart_min_x) / TRACK_LENGTH_METERS
                    PIXEL_CENTER     = 0.5 * (cart_min_x + cart_max_x)
                    print(f"[CALIBRATION] Complete. Setpoint y: {fixed_pivot_y}, Cart x-range: [{cart_min_x}, {cart_max_x}]")
                else:
                    fixed_pivot_y, cart_min_x, cart_max_x = None, None, None
                    print("[CALIBRATION] FAILED: No cart detections were made.")
        time.sleep(0.01)
    cv2.destroyAllWindows()


def main():
    start_zmq_server()

    capture = dv.io.CameraCapture()
    if not capture.isEventStreamAvailable():
        raise RuntimeError("The connected camera does not provide an event stream.")

    visualizer = dv.visualization.EventVisualizer(
        capture.getEventResolution()
    )
    visualizer.setBackgroundColor(
        dv.visualization.colors.white()
    )
    visualizer.setPositiveColor(
        dv.visualization.colors.iniBlue()
    )
    visualizer.setNegativeColor(
        dv.visualization.colors.darkGrey()
    )

    # seed last_cart
    shape = capture.getEventResolution()
    global last_cart_x, last_cart_y
    last_cart_x = shape[0]//2 if shape else 0
    last_cart_y = shape[1]//2 if shape else 0

    def slicing_callback(events: dv.EventStore):
        if events:
            result = process_events(events, visualizer)
            if result:
                with lock:
                    latest_detection.update(result)

    slicer = dv.EventStreamSlicer()
    slicer.doEveryNumberOfEvents(
        1000,
        slicing_callback
    )

    vis_thread = threading.Thread(
        target=visualisation_thread, daemon=True
    )
    vis_thread.start()

    print("Entering main loop… Press 'c' to calibrate, 'q' to quit.")
    while not quit_flag['quit']:
        if not capture.isRunning():
            time.sleep(0.1)
            continue
        events = capture.getNextEventBatch()
        if events is not None:
            slicer.accept(events)
        else:
            time.sleep(0.001)

    vis_thread.join()
    stop_zmq_server()
    print("Exited cleanly.")

if __name__ == "__main__":
    main()
