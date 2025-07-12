# /*----------------------------------------------------*\
# |                                                      |
# |         Telluride Neuromorphic Workshop              |
# |   Cartpole Control with Event-Based Vision Sensor    |
# |                                                      |
# \*----------------------------------------------------*/

# Import necessary libraries
import dv_processing as dv  # For event-based camera processing
import cv2  # OpenCV for image processing and display
import time  # For timing operations
from datetime import timedelta  # For specifying time intervals
import numpy as np  # For numerical operations, especially with arrays

from angle_pos_zmq import start_zmq_server, publish_estimate, stop_zmq_server  # NEW

# --- Global Variables ---
# Stores the y-coordinate of the user-defined horizontal line.
# Using a list is a common Python technique to make a variable mutable
# across different function scopes, allowing the callback to modify it.
user_line_y = [None]


# --- Mouse Callback Function ---
def mouse_callback(event, x, y, flags, param):
    """
    Handles mouse events in the OpenCV window. A left-click sets the
    position of the horizontal reference line.
    """
    # If the left mouse button is clicked, record the y-coordinate.
    if event == cv2.EVENT_LBUTTONDOWN:
        user_line_y[0] = y
        print(f"[INFO] User set horizontal line at y = {y}")


# --- Main Application Logic ---
def main():
    """
    Main function to run the event camera processing and visualization.
    """

    start_zmq_server()

    # --- Configuration ---
    # Time window for accumulating events into a single frame (in milliseconds).
    # This is also a list to allow modification from within the callback scope.
    integration_ms = [10]
    min_integration = 5      # Minimum allowed integration time.
    max_integration = 20    # Maximum allowed integration time.
    region_margin = 15       # Margin above/below the user's line to ignore during line detection.

    # --- Initialization ---
    # Set up the connection to the DVS camera.
    capture = dv.io.CameraCapture()
    if not capture.isEventStreamAvailable():
        raise RuntimeError("Input camera does not provide an event stream.")

    # Visualizer for converting event data into a viewable image.
    visualizer = dv.visualization.EventVisualizer(capture.getEventResolution())
    visualizer.setBackgroundColor(dv.visualization.colors.white())
    visualizer.setPositiveColor(dv.visualization.colors.iniBlue())
    visualizer.setNegativeColor(dv.visualization.colors.darkGrey())

    # Set up the display window and link the mouse callback function to it.
    cv2.namedWindow("Preview", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("Preview", mouse_callback)

    # Slicer to process events in fixed time intervals.
    slicer = dv.EventStreamSlicer()

    # --- State Variables ---
    # Flags and timers for controlling the main loop.
    quit_flag = {'quit': False}
    last_display = [time.time()]

    # --- Event Processing Callback ---
    def slicing_callback(events: dv.EventStore):
        """
        This function is called periodically to process accumulated events.
        It generates an image, detects lines, and updates the display.
        """
        # Allow modification of the outer scope's timer variable.
        nonlocal last_display
        now = time.time()
        
        # Calculate how long to wait between display updates, converting ms to seconds.
        display_interval = integration_ms[0] / 1000.0

        # Only update the display if enough time has passed since the last update.
        if now - last_display[0] >= display_interval:
            # --- Frame Generation ---
            # Generate an image from the accumulated events.
            frame = visualizer.generateImage(events)
            # Rotate the image for a vertical (portrait) orientation, typical for cart-pole.
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
            # Create a copy of the frame to draw visualizations on, leaving the original intact.
            vis = frame.copy()

            # --- Image Pre-processing ---
            # Convert to grayscale, then apply a binary threshold to create a clean black and white image.
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
            _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY)

            # If a user line is set, "erase" a band around it to prevent it from interfering with line detection.
            if user_line_y[0] is not None:
                y_user = int(user_line_y[0])
                y_start = max(0, y_user - region_margin)
                y_end = min(binary.shape[0], y_user + region_margin)
                # Set the region to white (255) so the Canny edge detector ignores it.
                binary[y_start:y_end, :] = 255

            # --- Line Detection ---
            # Use Canny to detect edges, then HoughLinesP to find lines in the edge map.
            edges = cv2.Canny(binary, 50, 150, apertureSize=3)
            lines = cv2.HoughLinesP(
                edges, 1, np.pi / 180,
                threshold=60, minLineLength=80, maxLineGap=20
            )

            # --- Analysis and Visualization ---
            intersection_x = None
            angle_from_vertical = None

            if lines is not None and len(lines) > 0:
                # Find the longest line segment among all detected lines using Euclidean distance.
                longest = max(
                    lines,
                    key=lambda l: (l[0][2] - l[0][0]) ** 2 + (l[0][3] - l[0][1]) ** 2
                )
                x1, y1, x2, y2 = map(float, longest[0])

                # Draw the longest detected line in red on the visualization image.
                cv2.line(vis, (int(round(x1)), int(round(y1))), (int(round(x2)), int(round(y2))), (0, 0, 255), 2)

                # --- Angle Calculation ---
                # Calculate the angle of the line relative to the vertical axis.
                if abs(x2 - x1) < 1e-3:
                    angle_from_vertical = 0.0
                elif abs(y2 - y1) < 1e-3:
                    angle_from_vertical = 90.0
                else:
                    angle_rad = np.arctan2(y2 - y1, x2 - x1)
                    angle_deg = np.degrees(angle_rad)
                    # Adjust angle to be relative to vertical (90 degrees) and normalize to [-180, 180].
                    angle_from_vertical = (angle_deg - 90) % 360
                    if angle_from_vertical > 180:
                        angle_from_vertical -= 360

                # --- Intersection Calculation ---
                # If the user has defined a line, calculate where the detected line intersects it.
                user_y = user_line_y[0]
                if user_y is not None:
                    # Check for vertical lines to avoid division by zero.
                    if abs(x2 - x1) < 1e-3:
                        intersection_x = int(round(x1))
                        if 0 <= intersection_x < vis.shape[1]:
                            cv2.circle(vis, (intersection_x, int(user_y)), 8, (0, 255, 255), -1)
                    # Check for horizontal lines (which won't intersect the user's horizontal line).
                    elif abs(y2 - y1) < 1e-3:
                        intersection_x = None
                    # If not vertical or horizontal, calculate intersection using line equation.
                    else:
                        m = (y2 - y1) / (x2 - x1)
                        intersection_x = int(round((user_y - y1) / m + x1))
                        # If the intersection is within the image bounds, draw a marker.
                        if 0 <= intersection_x < vis.shape[1]:
                            cv2.circle(vis, (intersection_x, int(user_y)), 8, (0, 255, 255), -1)
                    # Always draw the user-defined horizontal line in cyan.
                    cv2.line(vis, (0, int(user_y)), (vis.shape[1] - 1, int(user_y)), (255, 255, 0), 1)

                # Print the calculated information to the console.
                print(f"[INFO] Frame: Hough angle={angle_from_vertical:.1f} deg, x intersection={intersection_x}")
            
            else:
                # If no lines were detected, still draw the user line if it has been set.
                if user_line_y[0] is not None:
                    cv2.line(vis, (0, int(user_line_y[0])), (vis.shape[1] - 1, int(user_line_y[0])), (255, 255, 0), 1)

            publish_estimate(angle_from_vertical, intersection_x)

            # Display the final image with all visualizations.
            cv2.imshow("Preview", vis)

            # --- User Input ---
            # Handle keyboard commands for quitting or adjusting integration time.
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                quit_flag['quit'] = True
            elif key in (ord('+'), ord('=')):
                integration_ms[0] = min(max_integration, integration_ms[0] + 5)
            elif key == ord('-'):
                integration_ms[0] = max(min_integration, integration_ms[0] - 5)
            
            # Update the time of the last display.
            last_display[0] = now

    # Set up a callback to process event data in fixed time chunks.
    # The 'slicing_callback' function will be called for every 10-millisecond
    # interval of event data received from the camera.
    slicer.doEveryTimeInterval(timedelta(milliseconds=10), slicing_callback)

    # --- Main Loop ---
    # Continuously capture event batches from the camera and pass them to the slicer.
    while capture.isRunning() and not quit_flag['quit']:
        events = capture.getNextEventBatch()
        if events is not None:
            slicer.accept(events)

    # --- Cleanup ---
    stop_zmq_server()
    cv2.destroyAllWindows()
    capture.reset()

# --- Entry Point ---
# This ensures the main() function is called only when the script is executed directly.
if __name__ == "__main__":
    main()
