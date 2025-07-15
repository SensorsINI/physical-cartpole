# /*------------------------------------------------------*\
# | Telluride Neuromorphic Workshop: Cartpole TCP Output  |
# |          |
# |   - Calibrate setpoint y and cart x-ends with 'c'     |
# |   - Outputs cart_x, v, pole angle, pole omega, t      |
# |     every 10ms to console and TCP socket              |
# |   - No None values ever in output                     |
# |   - Visualization with color overlays                 |
# |                                                      |
# | 1. Press 'c' to start calibration. Move cart (circle) |
# |    to both ends.                                      |
# | 2. Press 'c' again to finish calibration.             |
# | 3. Data outputs every 10ms after calibration.         |
# | 4. Press 'q' to quit.                                 |
# \*------------------------------------------------------*/

import dv_processing as dv
import cv2
import time
from datetime import timedelta
import numpy as np
import threading
from angle_pos_zmq import start_zmq_server, publish_estimate, stop_zmq_server

TRACK_LENGTH_METERS = 0.44

# Shared state between threads
latest_detection = {
    "frame": None,
    "line": None,
    "circle_x": None,
    "circle_y": None,
    "angle": None,
    "timestamp": None
}

# Output values for TCP and console
output_values = {
    "cart_x": 0,  # <-- Start at 0 (will be replaced by center in main)
    "linear_velocity": 0.0,
    "angle": 0.0,
    "angular_velocity": 0.0,
    "timestamp": None
}

lock = threading.Lock()
quit_flag = {'quit': False}

# Calibration state
calibrating = False
calib_circle_xs = []
calib_circle_ys = []
fixed_pivot_y = None
cart_min_x = None
cart_max_x = None
BAND_HALF_HEIGHT = 50  # Visual only

PIXELS_PER_METER = None
PIXEL_CENTER = None

# For velocity/angle calculations
prev_cart_x = None
prev_cart_x_time = None
prev_angle = None
prev_angle_time = None

# --- NEW: Robust cart position state ---
last_cart_x = None
last_cart_y = None

# TCP output settings
TCP_HOST = '127.0.0.1'
TCP_PORT = 65432

time_start = time.time()

def process_events(events, visualizer):
    """
    Processes incoming event batches:
     - Detects lines for the pole (Hough, filtered)
     - Detects the cart (circle), with NO y-band limiting
     - Handles calibration accumulation (all detected circles)
     - Returns state dict for visualization and output
    """
    global calibrating, calib_circle_xs, calib_circle_ys, fixed_pivot_y
    global last_cart_x, last_cart_y

    frame = visualizer.generateImage(events)
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY)

    # --- Pole (line) detection with robust filtering ---
    edges = cv2.Canny(binary, 50, 150, apertureSize=3)
    lines = cv2.HoughLinesP(
        edges, 1, np.pi / 180,
        threshold=40, minLineLength=20, maxLineGap=10
    )
    line = None
    angle_from_vertical = None

    # Filter: ignore lines close to pivot-y after calibration
    filtered_lines = []
    if lines is not None and len(lines) > 0:
        if fixed_pivot_y is not None and not calibrating:
            for l in lines:
                x1, y1, x2, y2 = map(float, l[0])
                if abs(y1 - fixed_pivot_y) < 40 and abs(y2 - fixed_pivot_y) < 40:
                    continue
                filtered_lines.append(l)
        else:
            filtered_lines = lines

        # Pick the longest filtered line as pole
        if len(filtered_lines) > 0:
            longest_line = max(
                filtered_lines,
                key=lambda l: np.linalg.norm((l[0][2] - l[0][0], l[0][3] - l[0][1]))
            )
            # --- inside process_events(), after longest_line has been chosen ------------
            x1, y1, x2, y2 = map(float, longest_line[0])

            # Identify pivot (hinge point) and tip
            if fixed_pivot_y is not None:  # calibration available
                if abs(y1 - fixed_pivot_y) < abs(y2 - fixed_pivot_y):
                    pivot_x, pivot_y, tip_x, tip_y = x1, y1, x2, y2
                else:
                    pivot_x, pivot_y, tip_x, tip_y = x2, y2, x1, y1
            else:  # fallback: lower end is pivot
                if y1 > y2:  # y grows downward on screen
                    pivot_x, pivot_y, tip_x, tip_y = x1, y1, x2, y2
                else:
                    pivot_x, pivot_y, tip_x, tip_y = x2, y2, x1, y1

            line = (pivot_x, pivot_y, tip_x, tip_y)  # keep for drawing

            # 0 ° = upright, 90 ° = fallen to the *left*, 180 ° = upside‑down, 270 ° = right
            # after you have pivot_x, pivot_y, tip_x, tip_y  (see previous step)
            dx = tip_x - pivot_x  # screen‑space Δx
            dy = tip_y - pivot_y  # screen‑space Δy  (y grows downward!)

            # signed tilt:
            #   0°   = vertical up
            #   +90° = fallen to the *left*
            #   -90° = fallen to the *right*
            angle_from_vertical = np.degrees(np.arctan2(-dx, -dy))  # range [‑180, 180]

            # ---------------------------------------------------------------------------

    # --- Cart (circle) detection (NO y-axis limiting!) ---
    circle_x = None
    circle_y = None

    # Tuned parameters
    blur_ksize = 2
    if blur_ksize < 1: blur_ksize = 1
    if blur_ksize % 2 == 0: blur_ksize += 1
    blur = cv2.medianBlur(gray, blur_ksize)

    circles = cv2.HoughCircles(
        blur,
        cv2.HOUGH_GRADIENT,
        dp=1.5,
        minDist=15,
        param1=60,
        param2=20,
        minRadius=9,
        maxRadius=19
    )

    if circles is not None:
        circles_ = np.round(circles[0, :]).astype("int")
        x, y, r = circles_[0]
        circle_x, circle_y = x, y
        # --- NEW: Always remember last valid detection ---
        last_cart_x, last_cart_y = x, y
        if calibrating:
            calib_circle_xs.append(x)
            calib_circle_ys.append(y)
    else:
        # --- NEW: Always use last value if detection missed ---
        circle_x = last_cart_x if last_cart_x is not None else 0
        circle_y = last_cart_y if last_cart_y is not None else 0

    result = {
        "frame": frame,
        "line": line,
        "circle_x": circle_x,
        "circle_y": circle_y,
        "angle": angle_from_vertical,
        "timestamp": time.time()-time_start
    }
    return result

def visualisation_thread():
    """
    Displays live visualization and handles calibration state.
    Also calculates velocities and updates shared output_values.
    """
    global calibrating, calib_circle_xs, calib_circle_ys
    global fixed_pivot_y, cart_min_x, cart_max_x
    global prev_cart_x, prev_cart_x_time, prev_angle, prev_angle_time

    cv2.namedWindow("Preview", cv2.WINDOW_NORMAL)
    waiting_overlay = True
    while not quit_flag['quit']:
        with lock:
            frame = latest_detection["frame"]
            line = latest_detection["line"]
            circle_x = latest_detection["circle_x"]
            circle_y = latest_detection["circle_y"]
            angle_from_vertical = latest_detection["angle"]
            now = latest_detection["timestamp"]

        # --- Velocity and angle calculations after calibration ---
        linear_velocity = None
        angular_velocity = None
        if not calibrating and fixed_pivot_y is not None:
            # Linear velocity (pixels/sec)
            if circle_x is not None:
                if prev_cart_x is not None and prev_cart_x_time is not None:
                    dt = now - prev_cart_x_time
                    if dt > 0:
                        linear_velocity = (circle_x - prev_cart_x) / dt
                prev_cart_x = circle_x
                prev_cart_x_time = now
            # Angular velocity (deg/sec)
            if angle_from_vertical is not None:
                if prev_angle is not None and prev_angle_time is not None:
                    dt = now - prev_angle_time
                    if dt > 0:
                        diff = angle_from_vertical - prev_angle
                        diff = (diff + 180) % 360 - 180  # unwrap through the ±180 ° seam
                        angular_velocity = diff / dt
                prev_angle = angle_from_vertical
                prev_angle_time = now

            # Update output values for output thread (ALWAYS a number)
            with lock:
                output_values['cart_x'] = circle_x if circle_x is not None else 0
                output_values['linear_velocity'] = linear_velocity if linear_velocity is not None else 0.0
                output_values['angle'] = angle_from_vertical if angle_from_vertical is not None else 0.0
                output_values['angular_velocity'] = angular_velocity if angular_velocity is not None else 0.0
                output_values['timestamp'] = now

        # --- Draw overlays and calibration band ---
        if frame is not None:
            waiting_overlay = False
            vis = frame.copy()
            height, width = vis.shape[:2]
            center_x = width // 2
            cv2.line(vis, (center_x, 0), (center_x, height-1), (0, 140, 255), 2)
            center_y = height // 2
            # Green calibration band (visual only)
            if fixed_pivot_y is not None:
                y_min = max(0, fixed_pivot_y - BAND_HALF_HEIGHT)
                y_max = min(height-1, fixed_pivot_y + BAND_HALF_HEIGHT)
                cv2.rectangle(vis, (0, y_min), (width-1, y_max), (0, 255, 0), 1)
            else:
                y_min = max(0, center_y - BAND_HALF_HEIGHT)
                y_max = min(height-1, center_y + BAND_HALF_HEIGHT)
                cv2.rectangle(vis, (0, y_min), (width-1, y_max), (0, 255, 0), 1)
            # Magenta setpoint line
            if fixed_pivot_y is not None:
                cv2.line(vis, (0, fixed_pivot_y), (width-1, fixed_pivot_y), (255, 0, 255), 2)
            # Cyan endpoints for cart travel
            if cart_min_x is not None:
                cv2.line(vis, (cart_min_x, 0), (cart_min_x, height-1), (255, 255, 0), 2)
            if cart_max_x is not None:
                cv2.line(vis, (cart_max_x, 0), (cart_max_x, height-1), (255, 255, 0), 2)
            # Red: pole (detected line)
            if line is not None:
                x1, y1, x2, y2 = line
                cv2.line(vis, (int(round(x1)), int(round(y1))),
                         (int(round(x2)), int(round(y2))), (0, 0, 255), 2)
            # Green: cart (circle, at detected location)
            if circle_x is not None and circle_y is not None:
                cv2.circle(vis, (circle_x, circle_y), 10, (0, 255, 0), 2)
                cv2.circle(vis, (circle_x, circle_y), 8, (0, 255, 255), -1)
            # Calibration overlay text
            if calibrating:
                cv2.putText(vis, "CALIBRATING: move cart to both ends then press 'c' again",
                            (30, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
            cv2.imshow("Preview", vis)
        else:
            if waiting_overlay:
                black = np.zeros((240, 346, 3), dtype=np.uint8)
                cv2.putText(black, "Waiting for events...", (60, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
                cv2.imshow("Preview", black)
        # --- Calibration toggle: 'c' key ---
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            quit_flag['quit'] = True
        elif key == ord('c'):
            if not calibrating:
                print("[CALIBRATION] Starting calibration: move cart (circle) to both ends, press 'c' again.")
                calibrating = True
                calib_circle_xs = []
                calib_circle_ys = []
                prev_cart_x = None
                prev_cart_x_time = None
                prev_angle = None
                prev_angle_time = None
            else:
                if calib_circle_xs and calib_circle_ys:
                    fixed_pivot_y = int(np.mean(calib_circle_ys))
                    cart_min_x = int(np.min(calib_circle_xs))
                    cart_max_x = int(np.max(calib_circle_xs))
                    print(f"[CALIBRATION] Setpoint y: {fixed_pivot_y}, Cart min_x: {cart_min_x}, max_x: {cart_max_x}")
                else:
                    fixed_pivot_y = None
                    cart_min_x = None
                    cart_max_x = None
                    print("[CALIBRATION] No circles detected, calibration failed.")
                calibrating = False
        time.sleep(0.05)
    cv2.destroyAllWindows()

def output_thread():
    """
    Every 10ms after calibration, outputs values to console and a TCP server.
    Handles auto-reconnect for robust real-time streaming.
    All values are formatted with no None.
    """
    tcp_sock = None
    connected = False

    def safe(val, places=2, default=""):
        if val is None:
            return default
        if isinstance(val, float):
            return f"{val:.{places}f}"
        return str(val)

    while not quit_flag['quit']:
        if fixed_pivot_y is not None and not calibrating:
            with lock:
                cart_x = output_values['cart_x']
                v = output_values['linear_velocity']
                angle = output_values['angle']
                omega = output_values['angular_velocity']
                ts = output_values['timestamp']

                PIXELS_PER_METER = (cart_max_x - cart_min_x) / TRACK_LENGTH_METERS
                PIXEL_CENTER = 0.5 * (cart_max_x + cart_min_x)

                cart_x_meters = (cart_x - PIXEL_CENTER) / PIXELS_PER_METER
                linear_velocity_mps = v / PIXELS_PER_METER


            publish_estimate(
                angle_deg=np.deg2rad(angle),
                cart_x=cart_x_meters,
                linear_velocity=linear_velocity_mps,
                angular_velocity=omega,
                timestamp=ts,
            )
            # out_line = f"{safe(ts,3)},{safe(cart_x,0)},{safe(v)},{safe(angle,1)},{safe(omega)}\n"
            print(f"[DATA] t={safe(ts,3)} s; cart_x_meters={safe(cart_x_meters,3)} m; linear_velocity={safe(linear_velocity_mps)} m/s; angle={safe(angle,1)} deg; omega={safe(omega)} deg/s")
        time.sleep(0.01)

def main():
    """
    Initializes DV device, threads, and enters processing loop.
    Also sets last_cart_x/y to image center for safe default.
    """
    global last_cart_x, last_cart_y
    capture = dv.io.CameraCapture()
    if not capture.isEventStreamAvailable():
        raise RuntimeError("The connected camera does not provide an event stream.")

    visualizer = dv.visualization.EventVisualizer(capture.getEventResolution())
    visualizer.setBackgroundColor(dv.visualization.colors.white())
    visualizer.setPositiveColor(dv.visualization.colors.iniBlue())
    visualizer.setNegativeColor(dv.visualization.colors.darkGrey())

    # --- NEW: Set last_cart_x/y to image center as safe fallback
    shape = capture.getEventResolution()
    if shape is not None:
        last_cart_x = shape[0] // 2
        last_cart_y = shape[1] // 2
    else:
        last_cart_x = 0
        last_cart_y = 0

    start_zmq_server()

    slicer = dv.EventStreamSlicer()
    vis_thread = threading.Thread(
        target=visualisation_thread, daemon=True
    )
    vis_thread.start()
    out_thread = threading.Thread(
        target=output_thread, daemon=True
    )
    out_thread.start()

    def slicing_callback(events: dv.EventStore):
        if events is not None:
            result = process_events(events, visualizer)
            if result is not None:
                with lock:
                    latest_detection.update(result)

    slicer.doEveryTimeInterval(timedelta(milliseconds=5), slicing_callback)

    print("Entering main loop...")
    while not quit_flag['quit']:
        if not capture.isRunning():
            print("Camera not running or disconnected. Waiting for reconnect...")
            time.sleep(0.1)
            continue
        events = capture.getNextEventBatch()
        if events is not None:
            slicer.accept(events)
        else:
            time.sleep(0.01)
    vis_thread.join()
    out_thread.join()
    stop_zmq_server()

if __name__ == "__main__":
    main()
