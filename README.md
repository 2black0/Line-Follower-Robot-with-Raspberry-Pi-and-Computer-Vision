# 🤖 Line Follower Robot with Raspberry Pi and Computer Vision

A robust line-following robot system using a Raspberry Pi and Arduino-based microcontroller. This project employs computer vision via the Raspberry Pi camera (using `Picamera2`) and PID control to track and follow a black line on the floor in real-time.

---

## 🗂️ Project Structure

```

.
├── LICENSE
├── Project
│   ├── arduino
│   │   └── robot-control.ino         # Arduino sketch for motor driver control
│   └── python
│       └── computer-vision-control.py # Raspberry Pi-based vision and PID control
└── README.md

```

---

## 🎯 Objective

To design and implement a real-time line follower robot that:
- Detects black lines using computer vision (OpenCV + Picamera2),
- Calculates the deviation (error) from the line center,
- Controls wheel speeds using a PID controller, and
- Sends speed commands to an Arduino (or ESP32) via serial communication.

---

## 🧠 System Overview

### 📷 Vision Processing (Raspberry Pi)

- Uses Picamera2 with OpenCV to capture and process frames.
- Applies Otsu thresholding to isolate black lines.
- Defines a region of interest (ROI) on a selected horizontal line.
- Calculates the centroid of the line via image moments.

### ⚙️ Control Logic

- Computes error between image center and line center.
- Applies PID controller (`Kp`, `Ki`, `Kd`) to calculate speed delta.
- Adjusts left and right motor speeds accordingly.
- Sends formatted command via serial: `L<left_speed> R<right_speed>`

### 🛠️ Actuation (Arduino)

- Receives motor speed via serial from Raspberry Pi.
- Controls motor driver (e.g., L298N or BTS7960) using PWM.
- Parses simple serial commands to drive motors left/right.

---

## 🚀 How It Works

### 1. **Run Python Code on Raspberry Pi**
```bash
cd Project/python
python3 computer-vision-control.py
```

### 2. **Upload Arduino Code**

Use the Arduino IDE to upload `robot-control.ino` to the microcontroller.

### 3. **Wiring Overview**

* Camera → Raspberry Pi CSI port
* Raspberry Pi USB → ESP32/Arduino serial
* Motor driver → Arduino PWM outputs
* Motors → Motor driver outputs
* Power → Independent source (battery)

---

## 🔧 Configuration

You can tune the following parameters in `computer-vision-control.py`:

```python
self.Kp = 0.25    # Proportional gain
self.Ki = 0.0     # Integral gain
self.Kd = 0.0     # Derivative gain

self.base_speed = 150  # Nominal motor speed
self.min_speed = 75    # Minimum motor speed
self.max_speed = 250   # Maximum motor speed

self.roi_row = 7       # Row index (out of 10) for line detection
self.min_area = 500    # Minimum contour area to reduce noise
```

---

## 📦 Dependencies

Install dependencies on Raspberry Pi:

```bash
sudo apt update
sudo apt install python3-opencv python3-picamera2
pip install pyserial numpy
```

---

## 🧪 PID Tuning

You can adjust PID gains dynamically for:

* **Fast response** → Increase `Kp`
* **Less oscillation** → Add small `Kd`
* **Correct steady error** → Add small `Ki` (carefully)

---

## 📸 Example Output

* Red dot: detected line center
* Blue rectangle: ROI boundary
* Yellow line: image center (reference)
* Green overlay: valid line region

---

## 🛡️ License

This project is licensed under the MIT License. See the [LICENSE](../LICENSE) file for details.

---

## 🙌 Acknowledgements

* Built on Raspberry Pi OS with Picamera2.
* Real-time line following using OpenCV and Arduino motor control.