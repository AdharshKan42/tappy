import cv2
import numpy as np
import pyrealsense2 as rs  # Import the RealSense SDK

def nothing(x):
    pass

if __name__ == "__main__":
    # Initialize RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

    try:
        pipeline.start(config)
    except Exception as e:
        print(f"Error: Could not start RealSense pipeline. {e}")
        exit()

    cv2.namedWindow("Real-time Mask Adjustment")
    cv2.createTrackbar("Lower H", "Real-time Mask Adjustment", 0, 179, nothing)
    cv2.createTrackbar("Lower S", "Real-time Mask Adjustment", 0, 255, nothing)
    cv2.createTrackbar("Lower V", "Real-time Mask Adjustment", 0, 255, nothing)
    cv2.createTrackbar("Upper H", "Real-time Mask Adjustment", 179, 179, nothing)
    cv2.createTrackbar("Upper S", "Real-time Mask Adjustment", 255, 255, nothing)
    cv2.createTrackbar("Upper V", "Real-time Mask Adjustment", 255, 255, nothing)

    while(True):
        # Capture frames from RealSense
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            print("Error: Could not retrieve color frame.")
            break

        frame = np.asanyarray(color_frame.get_data())

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_h = cv2.getTrackbarPos("Lower H", "Real-time Mask Adjustment")
        lower_s = cv2.getTrackbarPos("Lower S", "Real-time Mask Adjustment")
        lower_v = cv2.getTrackbarPos("Lower V", "Real-time Mask Adjustment")
        upper_h = cv2.getTrackbarPos("Upper H", "Real-time Mask Adjustment")
        upper_s = cv2.getTrackbarPos("Upper S", "Real-time Mask Adjustment")
        upper_v = cv2.getTrackbarPos("Upper V", "Real-time Mask Adjustment")

        lower_tan = np.array([lower_h, lower_s, lower_v])
        upper_tan = np.array([upper_h, upper_s, upper_v])

        mask = cv2.inRange(hsv_frame, lower_tan, upper_tan)
        result = cv2.bitwise_and(frame, frame, mask=mask)

        cv2.imshow("Original Camera Feed", frame)
        cv2.imshow("Real-time Mask", mask)
        cv2.imshow("Real-time Result", result)

        k = cv2.waitKey(1) & 0xFF
        if k == 27: # Press Esc to exit
            break

    pipeline.stop()
    cv2.destroyAllWindows()

    print(f"Final Lower Tan HSV: [{lower_h}, {lower_s}, {lower_v}]")
    print(f"Final Upper Tan HSV: [{upper_h}, {upper_s}, {upper_v}]")