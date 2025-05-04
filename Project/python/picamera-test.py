import cv2
import numpy as np
from picamera2 import Picamera2
import serial  # For serial communication with ESP32

class LineDetectionCamera:
    def __init__(self, width=640, height=480, roi_row=7, min_area=500, serial_port='/dev/ttyUSB0', baud_rate=115200):
        self.width = width
        self.height = height
        self.roi_row = roi_row  # Default ROI row (7th from the top)
        self.min_area = min_area  # Minimum area for valid contours (to filter noise)
        self.cam = Picamera2()
        self.cam.configure(self.cam.create_video_configuration(main={"format": 'RGB888', "size": (width, height)}))
        self.running = False

        # PID parameters
        self.Kp = 0.25  # Adjusted proportional gain for smoother response
        self.Ki = 0.0
        self.Kd = 0.0
        self.previous_error = 0
        self.integral = 0

        # Motor speed range
        self.base_speed = 150
        self.min_speed = 75
        self.max_speed = 250

        # Serial connection
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.serial_conn = None

    def start_serial(self):
        """Start the serial connection to ESP32."""
        try:
            self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            print(f"Serial connection established on {self.serial_port} at {self.baud_rate} baud.")
        except Exception as e:
            print(f"Failed to connect to {self.serial_port}: {e}")
            self.serial_conn = None

    def send_motor_speeds(self, left_speed, right_speed):
        """Send motor speeds to ESP32 over serial."""
        if self.serial_conn and self.serial_conn.is_open:
            try:
                # Format the motor speed as "L<left_speed> R<right_speed>\n"
                message = f"L{int(left_speed)} R{int(right_speed)}\n"
                self.serial_conn.write(message.encode())
                print(f"Sent to ESP32: {message.strip()}")
            except Exception as e:
                print(f"Failed to send data: {e}")


    def pid_control(self, error):
        """PID control to calculate the delta speed for motor adjustment."""
        # Proportional term
        proportional = self.Kp * error

        # Integral term
        self.integral += error
        integral = self.Ki * self.integral

        # Derivative term
        derivative = self.Kd * (error - self.previous_error)

        # Total PID output (delta speed)
        delta_speed = proportional + integral + derivative

        # Update previous error
        self.previous_error = error

        return delta_speed

    def start_camera(self):
        """Start the camera."""
        self.cam.start()
        self.running = True

    def stop_camera(self):
        """Stop the camera."""
        self.cam.stop()
        self.running = False

    def process_frame(self, frame):
        """Process the frame to detect black lines and calculate the line center."""
        # Flip the frame upside-down and left-right
        flipped_frame = cv2.flip(frame, -1)

        # Define the ROI dynamically based on the selected row
        row_height = self.height // 10  # Divide the height into 10 equal parts
        roi_start = self.roi_row * row_height
        roi_end = roi_start + row_height
        roi = flipped_frame[roi_start:roi_end, 0:self.width]

        # Convert ROI to grayscale
        gray_frame = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        # Apply Otsu's thresholding
        _, otsu_threshold = cv2.threshold(gray_frame, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # Invert the threshold to make the black line black and the background white
        inverted_threshold = cv2.bitwise_not(otsu_threshold)

        # Remove noise: filter out small contours based on area
        contours, _ = cv2.findContours(inverted_threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filtered_mask = np.zeros_like(inverted_threshold)  # Create a blank mask
        for contour in contours:
            if cv2.contourArea(contour) > self.min_area:
                cv2.drawContours(filtered_mask, [contour], -1, 255, thickness=cv2.FILLED)

        # Calculate the center of the black line
        moments = cv2.moments(filtered_mask)
        line_center_x = None  # Initialize line center to None
        if moments["m00"] > 0:  # Ensure there's a detected line
            line_center_x = int(moments["m10"] / moments["m00"])
            # Draw the detected center on the ROI
            cv2.circle(roi, (line_center_x, row_height // 2), 5, (0, 0, 255), -1)  # Red dot at the center

        # Overlay the filtered ROI back onto the flipped frame with transparency
        overlay = flipped_frame.copy()
        overlay[roi_start:roi_end, 0:self.width][filtered_mask == 255] = (0, 255, 0)  # Green overlay
        cv2.addWeighted(overlay, 0.5, flipped_frame, 0.5, 0, flipped_frame)  # Apply transparency

        # Draw the ROI boundary on the flipped frame
        cv2.rectangle(flipped_frame, (0, roi_start), (self.width, roi_end), (255, 0, 0), 2)  # Blue box

        # Draw the vertical center line on the flipped frame
        image_center_x = self.width // 2
        cv2.line(flipped_frame, (image_center_x, 0), (image_center_x, self.height), (255, 255, 0), 2)  # Yellow line

        # Calculate the error between the image center and the line center
        error = None
        if line_center_x is not None:
            error = line_center_x - image_center_x  # Positive if the line is to the right, negative if to the left

        return flipped_frame, error

    def run(self):
        """Main loop to capture, process, and control motors."""
        self.start_camera()
        self.start_serial()  # Initialize the serial connection
        try:
            while self.running:
                frame = self.cam.capture_array()

                # Process the frame for line detection
                processed_frame, error = self.process_frame(frame)

                # Display the processed frame
                cv2.imshow('Line Detection', processed_frame)

                # Control logic
                if error is not None:
                    # Calculate delta speed using PID control
                    delta_speed = self.pid_control(error)

                    # Calculate motor speeds (reversed logic)
                    left_motor_speed = self.base_speed + delta_speed
                    right_motor_speed = self.base_speed - delta_speed

                    # Clamp speeds within the valid range
                    left_motor_speed = max(self.min_speed, min(self.max_speed, left_motor_speed))
                    right_motor_speed = max(self.min_speed, min(self.max_speed, right_motor_speed))

                    # Send motor speeds to ESP32
                    self.send_motor_speeds(left_motor_speed, right_motor_speed)
                else:
                    print("No line detected. Stopping motors.")
                    self.send_motor_speeds(0, 0)  # Send stop command to motors

                # Exit on pressing 'q'
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            self.cleanup()

    def cleanup(self):
        """Cleanup resources."""
        self.stop_camera()
        if self.serial_conn:
            self.serial_conn.close()
        cv2.destroyAllWindows()

def main():
    print("Starting Line Detection Application. Press 'q' to quit.")
    print("The frame is divided into 10 rows for ROI selection.")
    print("You can change the `roi_row` parameter to select a specific region.")
    print("You can adjust the `min_area` parameter to filter out small noise.")
    app = LineDetectionCamera(width=640, height=480, roi_row=7, min_area=500)  # Adjust `min_area` as needed
    app.run()
    print("Application closed.")

if __name__ == "__main__":
    main()
