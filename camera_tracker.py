from picamera2 import Picamera2
import cv2
import numpy as np
import time

WIDTH = 160
HEIGHT = 120

# Yellow/orange ball HSV range
LOWER_ORANGE = np.array([15, 120, 120])
UPPER_ORANGE = np.array([30, 255, 255])

MIN_AREA = 20


class BallTracker:
    def __init__(self, show_windows=False):
        self.show_windows = show_windows

        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(
            main={"size": (WIDTH, HEIGHT), "format": "RGB888"}
        )
        self.picam2.configure(config)

        self.picam2.start()
        time.sleep(2)

        # Let auto white balance settle, then freeze it
        self.picam2.set_controls({
            "AwbEnable": False
        })

        time.sleep(1)

    def get_ball_position(self):
        frame = self.picam2.capture_array()

        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, LOWER_ORANGE, UPPER_ORANGE)

        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        center_x = WIDTH // 2
        center_y = HEIGHT // 2

        ball_position = None
        valid_contours = []

        for c in contours:
            area = cv2.contourArea(c)

            if area < MIN_AREA:
                continue

            perimeter = cv2.arcLength(c, True)

            if perimeter == 0:
                continue

            circularity = 4 * np.pi * area / (perimeter * perimeter)

            (x, y), radius = cv2.minEnclosingCircle(c)
            circle_area = np.pi * radius * radius
            fill_ratio = area / circle_area if circle_area > 0 else 0

            if circularity > 0.35 and fill_ratio > 0.25:
                valid_contours.append(c)

        if valid_contours:
            largest = max(valid_contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > MIN_AREA:
                (x, y), radius = cv2.minEnclosingCircle(largest)

                x = int(x)
                y = int(y)
                radius = int(radius)

                error_x = x - center_x
                error_y = center_y - y

                ball_position = (error_x, error_y)

                if self.show_windows:
                    cv2.circle(frame_bgr, (x, y), radius, (0, 255, 0), 2)
                    cv2.circle(frame_bgr, (x, y), 4, (0, 255, 0), -1)
                    cv2.putText(
                        frame_bgr,
                        f"x={error_x}, y={error_y}",
                        (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2
                    )

        if self.show_windows:
            cv2.drawMarker(
                frame_bgr,
                (center_x, center_y),
                (255, 255, 255),
                cv2.MARKER_CROSS,
                25,
                2
            )

            cv2.imshow("Ball Tracking", frame_bgr)
            cv2.imshow("Mask", mask)
            cv2.waitKey(1)

        return ball_position

    def stop(self):
        self.picam2.stop()
        cv2.destroyAllWindows()