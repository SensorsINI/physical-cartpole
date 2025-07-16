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

from Driver.DriverFunctions.DVS.angle_pos_zmq import start_zmq_server, stop_zmq_server, publish_estimate

# --- USER TUNED MATCH FILTER PARAMETERS ---
CART_RADIUS = 9      # px, radius of template for match filter
CART_THRESH = 0.50   # threshold for match quality (0-1)
ROI_BAND = 40        # px, half-height of ROI after calibration

TRACK_LENGTH_METERS = 0.44

# --- Global/Shared State ---
latest_detection = {
    "frame": None, "line": None, "cart_x": None, "cart_y": None,
    "angle": None, "timestamp": None
}
output_values = {
    "cart_x": 0, "linear_velocity": 0.0,
    "angle": 0.0, "angular_velocity": 0.0, "timestamp": None
}
lock = threading.Lock()
quit_flag = {'quit': False}

# --- Calibration state ---
calibrating = False
calib_cart_xs = []
calib_cart_ys = []
fixed_pivot_y = None
cart_min_x = None
cart_max_x = None
last_cart_x = None
last_cart_y = None
prev_cart_x = None
prev_cart_x_time = None
prev_angle = None
prev_angle_time = None
last_angle = None
PIXELS_PER_METER = None
PIXEL_CENTER    = None

def make_circle_template(radius, thickness=-1, image_size=None):
    """Create a circular template for match filtering."""
    size = 2 * radius + 5 if image_size is None else image_size
    template = np.zeros((size, size), dtype=np.uint8)
    center = (size // 2, size // 2)
    cv2.circle(template, center, radius, 255, thickness)
    return template

# ───────────────────────────── helpers ──────────────────────────────
def line_horizontal_intersect(x1, y1, x2, y2, y_horiz):
    """
    Return x-coordinate of intersection between the segment (x1,y1)-(x2,y2)
    and the horizontal line y = y_horiz **iff** the intersection lies between
    the endpoints; otherwise return None.
    """
    # Segment is (almost) horizontal → no reliable crossing
    if abs(y2 - y1) < 1e-3:
        return None
    t = (y_horiz - y1) / float(y2 - y1)      # barycentric coordinate
    if 0.0 <= t <= 1.0:
        return x1 + t * (x2 - x1)
    return None


def gaussian_blur_uint8(img):
    """A tiny wrapper to keep the original Gaussian parameters in one place."""
    return cv2.GaussianBlur(img, (3, 3), 0)


def process_events(events, visualizer):
    """
    Processes one event batch:
      - During calibration, runs circle detection only to accumulate samples for pivot y.
      - After calibration, uses Hough-line intersection exclusively to compute pivot and cart position.
      - Filters *all* candidate lines by true intersection with the pivot, then picks the longest.
      - Falls back to last valid angle when no detection survives.
      - Returns a dict with:
        frame, line (pivot→tip), cart_x, cart_y, angle (deg), timestamp
    """
    global calibrating, calib_cart_xs, calib_cart_ys, fixed_pivot_y
    global last_cart_x, last_cart_y, last_angle

    # render and preprocess
    frame = visualizer.generateImage(events)
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    H, W  = gray.shape

    # ── Calibration: circle matcher only ────────────────────────────
    if calibrating or fixed_pivot_y is None:
        template = make_circle_template(CART_RADIUS)
        blurred  = gaussian_blur_uint8(gray)
        result   = cv2.matchTemplate(blurred, template, cv2.TM_CCOEFF_NORMED)
        _, conf, _, loc = cv2.minMaxLoc(result)

        if conf >= CART_THRESH:
            cx = loc[0] + template.shape[1] // 2
            cy = loc[1] + template.shape[0] // 2
            last_cart_x, last_cart_y = cx, cy
            if calibrating:
                calib_cart_xs.append(cx)
                calib_cart_ys.append(cy)
        else:
            cx, cy = last_cart_x or (W//2), last_cart_y or (H//2)

        return {
            "frame":    frame,
            "line":     None,
            "cart_x":   cx,
            "cart_y":   cy,
            "angle":    None,
            "timestamp": time.time()
        }

    # ── Runtime: Hough + *all‑lines* intersection filtering ─────────
    _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY)
    edges     = cv2.Canny(binary, 50, 150, apertureSize=3)
    lines     = cv2.HoughLinesP(edges, 1, np.pi/180,
                                threshold=40, minLineLength=20, maxLineGap=10)

    line      = None
    angle_deg = None
    pivot_x   = None
    pivot_y   = fixed_pivot_y

    if lines is not None:
        best_len = 0
        best_seg = None

        # 1) Test *every* segment for a real crossing at pivot_y
        for L in lines:
            x1, y1, x2, y2 = map(float, L[0])
            px = line_horizontal_intersect(x1, y1, x2, y2, pivot_y)
            if px is None:
                continue  # no true intersection within segment

            length = np.hypot(x2 - x1, y2 - y1)
            if length > best_len:
                best_len = length
                best_seg = (x1, y1, x2, y2, px)

        # 2) If at least one line crossed the pivot, pick the longest
        if best_seg is not None:
            x1, y1, x2, y2, px = best_seg
            pivot_x = px

            # choose tip as endpoint farthest from the pivot
            if np.hypot(x1 - px, y1 - pivot_y) > np.hypot(x2 - px, y2 - pivot_y):
                tip_x, tip_y = x1, y1
            else:
                tip_x, tip_y = x2, y2

            line      = (pivot_x, pivot_y, tip_x, tip_y)
            dx, dy    = tip_x - pivot_x, tip_y - pivot_y

            # compute angle from vertical (deg)
            angle_deg = np.degrees(np.arctan2(-dx, -dy))

    # ── Fallback: hold last valid angle if no new detection ─────────
    if angle_deg is not None:
        last_angle = angle_deg
    else:
        angle_deg = last_angle

    # enforce cart_x == pivot_x (or reuse last_cart_x)
    cx = pivot_x if pivot_x is not None else last_cart_x
    last_cart_x, last_cart_y = cx, pivot_y

    return {
        "frame":     frame,
        "line":      line,
        "cart_x":    cx,
        "cart_y":    pivot_y,
        "angle":     angle_deg,
        "timestamp": time.time()
    }




def visualisation_thread():
    """
    Handles display and calibration logic.
    Draws green cart circle at actual detected y (never fixed).
    """
    global calibrating, calib_cart_xs, calib_cart_ys, fixed_pivot_y, cart_min_x, cart_max_x
    global prev_cart_x, prev_cart_x_time, prev_angle, prev_angle_time

    cv2.namedWindow("Preview", cv2.WINDOW_NORMAL)
    waiting_overlay = True
    while not quit_flag['quit']:
        with lock:
            frame = latest_detection["frame"]
            line = latest_detection["line"]
            cart_x = latest_detection["cart_x"]
            cart_y = latest_detection["cart_y"]
            angle_from_vertical = latest_detection["angle"]
            now = latest_detection["timestamp"]

        # Raw velocity calculation (diff, not filtered)
        linear_velocity = 0.0
        if not calibrating and fixed_pivot_y is not None:
            if cart_x is not None and prev_cart_x is not None and prev_cart_x_time is not None:
                dt = now - prev_cart_x_time
                if dt > 0:
                    linear_velocity = (cart_x - prev_cart_x) / dt
            prev_cart_x = cart_x
            prev_cart_x_time = now

        # Raw angular velocity
        angular_velocity = 0.0
        if not calibrating and fixed_pivot_y is not None:
            if angle_from_vertical is not None and prev_angle is not None and prev_angle_time is not None:
                dt = now - prev_angle_time
                if dt > 0:
                    angular_velocity = (angle_from_vertical - prev_angle) / dt
            prev_angle = angle_from_vertical
            prev_angle_time = now

        # Output values (always numbers)
        # Only update when we actually detected something
        with lock:
            if cart_x is not None:
                output_values['cart_x'] = int(cart_x)
            if prev_cart_x_time is not None:
                output_values['linear_velocity'] = float(linear_velocity)
            if angle_from_vertical is not None:
                output_values['angle'] = float(angle_from_vertical)
            if prev_angle_time is not None:
                output_values['angular_velocity'] = float(angular_velocity)
            # always update timestamp so downstream threads know “freshness”
            output_values['timestamp'] = now

        # --- Draw overlays ---
        if frame is not None:
            waiting_overlay = False
            vis = frame.copy()
            height, width = vis.shape[:2]
            # ROI band
            if fixed_pivot_y is not None:
                y_min = max(0, fixed_pivot_y - ROI_BAND)
                y_max = min(height-1, fixed_pivot_y + ROI_BAND)
                cv2.rectangle(vis, (0, y_min), (width-1, y_max), (0, 255, 0), 1)
            # Red pole
            if line is not None:
                x1, y1, x2, y2 = line
                cv2.line(vis, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
            # Green cart (always at detected y)
            if cart_x is not None and cart_y is not None:
                cv2.circle(vis, (int(cart_x), int(cart_y)), CART_RADIUS, (0, 255, 0), 2)
            if calibrating:
                cv2.putText(vis, "CALIBRATING: move cart to ends, press 'c'", (30, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
            cv2.imshow("Preview", vis)
        else:
            if waiting_overlay:
                black = np.zeros((240, 346, 3), dtype=np.uint8)
                cv2.putText(black, "Waiting for events...", (60, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
                cv2.imshow("Preview", black)

        # Keyboard: calibration
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            quit_flag['quit'] = True
        elif key == ord('c'):
            if not calibrating:
                print("[CALIBRATION] Starting: move cart to both ends, then press 'c' again.")
                calibrating = True
                calib_cart_xs.clear()
                calib_cart_ys.clear()
            else:
                calibrating = False
                global PIXEL_CENTER, PIXELS_PER_METER
                if calib_cart_xs:
                    fixed_pivot_y = int(np.median(calib_cart_ys))
                    cart_min_x = int(np.min(calib_cart_xs))
                    cart_max_x = int(np.max(calib_cart_xs))
                    PIXELS_PER_METER = (cart_max_x - cart_min_x) / TRACK_LENGTH_METERS
                    PIXEL_CENTER = 0.5 * (cart_min_x + cart_max_x)
                    print(f"[CALIBRATION] Complete. Setpoint y: {fixed_pivot_y}, Cart x-range: [{cart_min_x}, {cart_max_x}]")
                else:
                    fixed_pivot_y, cart_min_x, cart_max_x = None, None, None
                    print("[CALIBRATION] FAILED: No cart detections were made.")
        time.sleep(0.01)
    cv2.destroyAllWindows()

def output_thread():
    """
    Outputs raw detected cart x/velocity, pole angle/angular velocity via ZeroMQ and console.
    """
    def safe(val, places=2, default=""): return f"{val:.{places}f}" if isinstance(val, float) else str(val) if val is not None else default
    while not quit_flag['quit']:
        if fixed_pivot_y is not None and not calibrating:
            with lock:
                cart_x = output_values['cart_x']
                v = output_values['linear_velocity']
                angle = output_values['angle']
                omega = output_values['angular_velocity']
                ts = output_values['timestamp']

            cart_x_m = (cart_x - PIXEL_CENTER) / PIXELS_PER_METER
            linear_velocity_mps = v / PIXELS_PER_METER

            # Publish via ZMQ
            publish_estimate(
                angle_rad=np.deg2rad(angle),
                cart_x=cart_x_m,
                linear_velocity=linear_velocity_mps,
                angular_velocity=np.deg2rad(omega),
                timestamp=ts,
            )

            out_line = f"{safe(ts, 3)},{safe(cart_x, 0)},{safe(v)},{safe(angle, 1)},{safe(omega)}\n"
            print(f"[DATA] t={safe(ts, 3)} cart_x={safe(cart_x, 0)} v={safe(v)} angle={safe(angle, 1)} omega={safe(omega)}")

        time.sleep(0.01)

def main():
    """
    Initializes DVS camera, visualizer, threads, and starts event-processing loop.
    """

    start_zmq_server()

    global last_cart_x, last_cart_y
    capture = dv.io.CameraCapture()
    if not capture.isEventStreamAvailable():
        raise RuntimeError("The connected camera does not provide an event stream.")

    visualizer = dv.visualization.EventVisualizer(capture.getEventResolution())
    visualizer.setBackgroundColor(dv.visualization.colors.white())
    visualizer.setPositiveColor(dv.visualization.colors.iniBlue())
    visualizer.setNegativeColor(dv.visualization.colors.darkGrey())

    shape = capture.getEventResolution()
    last_cart_x = shape[0] // 2 if shape else 0
    last_cart_y = shape[1] // 2 if shape else 0

    slicer = dv.EventStreamSlicer()
    def slicing_callback(events: dv.EventStore):
        if events:
            result = process_events(events, visualizer)
            if result:
                with lock:
                    latest_detection.update(result)
    slicer.doEveryTimeInterval(timedelta(milliseconds=5), slicing_callback)

    vis_thread = threading.Thread(target=visualisation_thread, daemon=True)
    vis_thread.start()
    out_thread = threading.Thread(target=output_thread, daemon=True)
    out_thread.start()

    print("Entering main loop... Press 'c' to calibrate, 'q' to quit.")
    while not quit_flag['quit']:
        if not capture.isRunning():
            print("Camera not running or disconnected. Waiting...")
            time.sleep(0.1)
            continue
        events = capture.getNextEventBatch()
        if events is not None:
            slicer.accept(events)
        else:
            time.sleep(0.001)
    print("Quit signal received. Shutting down threads...")
    vis_thread.join()
    out_thread.join()
    stop_zmq_server()
    print("Exited cleanly.")

if __name__ == "__main__":
    main()
