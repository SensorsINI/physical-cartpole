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
TCP_HOST = '127.0.0.1'
TCP_PORT = 65432

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
fixed_pivot_y = None
cart_min_x = None
cart_max_x = None
last_cart_x = None
last_cart_y = None
prev_cart_x = None
prev_cart_x_time = None
prev_angle = None
prev_angle_time = None

def make_circle_template(radius, thickness=-1, image_size=None):
    """Create a circular template for match filtering."""
    size = 2 * radius + 5 if image_size is None else image_size
    template = np.zeros((size, size), dtype=np.uint8)
    center = (size // 2, size // 2)
    cv2.circle(template, center, radius, 255, thickness)
    return template

def process_events(events, visualizer):
    """
    Processes one event batch:
      - Detects pole via Hough lines (raw angle, no filtering)
      - Detects cart using match filter (x, y), never fixed on y
      - Handles calibration accumulation (x values)
      - Returns results for vis/output
    """
    global calibrating, calib_cart_xs, fixed_pivot_y
    global last_cart_x, last_cart_y

    frame = visualizer.generateImage(events)
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    height, width = gray.shape

    # --- Pole detection via Hough ---
    _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY)
    edges = cv2.Canny(binary, 50, 150, apertureSize=3)
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=40, minLineLength=20, maxLineGap=10)
    line = None
    angle_from_vertical = None
    if lines is not None:
        # Use the longest line
        longest_line = max(lines, key=lambda l: np.linalg.norm((l[0][2] - l[0][0], l[0][3] - l[0][1])))
        x1, y1, x2, y2 = map(float, longest_line[0])
        line = (x1, y1, x2, y2)
        angle_rad = np.arctan2(y2 - y1, x2 - x1)
        angle_deg = np.degrees(angle_rad)
        angle_from_vertical = ((angle_deg - 90 + 180) % 360) - 180

    # --- Cart detection: match filter (never fixed y) ---
    template = make_circle_template(CART_RADIUS)
    blurred = cv2.GaussianBlur(gray, (3,3), 0)

    if fixed_pivot_y is not None and not calibrating:
        y0 = max(0, fixed_pivot_y - ROI_BAND)
        y1 = min(height, fixed_pivot_y + ROI_BAND)
        roi = blurred[y0:y1, :]
        roi_origin = (0, y0)
    else:
        roi = blurred
        roi_origin = (0, 0)

    result = cv2.matchTemplate(roi, template, cv2.TM_CCOEFF_NORMED)
    _, max_val, _, max_loc = cv2.minMaxLoc(result)
    if max_val > CART_THRESH:
        cart_x = roi_origin[0] + max_loc[0] + template.shape[1] // 2
        cart_y = roi_origin[1] + max_loc[1] + template.shape[0] // 2
        last_cart_x, last_cart_y = cart_x, cart_y
        if calibrating:
            calib_cart_xs.append(cart_x)
    else:
        cart_x = last_cart_x if last_cart_x is not None else width // 2
        cart_y = last_cart_y if last_cart_y is not None else height // 2

    # --- Return for visualization and output
    return {
        "frame": frame, "line": line,
        "cart_x": cart_x, "cart_y": cart_y,
        "angle": angle_from_vertical,
        "timestamp": time.time()
    }

def visualisation_thread():
    """
    Handles display and calibration logic.
    Draws green cart circle at actual detected y (never fixed).
    """
    global calibrating, calib_cart_xs, fixed_pivot_y, cart_min_x, cart_max_x
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
        with lock:
            output_values['cart_x'] = int(cart_x) if cart_x is not None else 0
            output_values['linear_velocity'] = float(linear_velocity)
            output_values['angle'] = float(angle_from_vertical) if angle_from_vertical is not None else 0.0
            output_values['angular_velocity'] = float(angular_velocity)
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
            else:
                calibrating = False
                if calib_cart_xs:
                    fixed_pivot_y = int(np.median([cart_y for _ in range(5)]))
                    cart_min_x = int(np.min(calib_cart_xs))
                    cart_max_x = int(np.max(calib_cart_xs))
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

            # Publish via ZMQ
            publish_estimate(
                angle_rad=np.deg2rad(angle),
                cart_x=cart_x,
                linear_velocity=v,
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
