from picamera2 import Picamera2
import cv2
import numpy as np
import time
import serial

# -------------------------
# CONFIGURATION
# -------------------------

PORT = "/dev/ttyACM0"
BAUD = 115200

# Same ROI used in your successful camera test
x1, y1 = 220, 140
x2, y2 = 420, 340

MIN_COVERAGE = 20.0

ranges = {
    "RED": [
        ((0, 100, 80), (10, 255, 255)),
        ((170, 100, 80), (179, 255, 255))
    ],
    "GREEN": [
        ((35, 70, 70), (85, 255, 255))
    ],
    "BLUE": [
        ((90, 70, 70), (135, 255, 255))
    ]
}

destinations = {
    "RED": "ZONE_A",
    "GREEN": "ZONE_B",
    "BLUE": "ZONE_C"
}

# -------------------------
# CAMERA
# -------------------------

camera = Picamera2()
camera.configure(
    camera.create_preview_configuration(
        main={"size": (640, 480), "format": "RGB888"}
    )
)

camera.start()
time.sleep(2)

try:
    # Capture one frame, as in your original test
    frame = camera.capture_array()

finally:
    camera.stop()

roi = frame[y1:y2, x1:x2]

# Your original working conversion
hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

scores = {}

for color, intervals in ranges.items():
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

    for lower, upper in intervals:
        mask |= cv2.inRange(
            hsv,
            np.array(lower, dtype=np.uint8),
            np.array(upper, dtype=np.uint8)
        )

    scores[color] = cv2.countNonZero(mask)

total_pixels = roi.shape[0] * roi.shape[1]

percentages = {
    color: 100 * count / total_pixels
    for color, count in scores.items()
}

detected_color = max(percentages, key=percentages.get)
confidence = percentages[detected_color]

if confidence < MIN_COVERAGE:
    detected_color = "UNKNOWN"

print("\nColor percentages:")

for color, percentage in percentages.items():
    print(f"{color}: {percentage:.1f}%")

print("Detected color:", detected_color)
print(
    "Destination:",
    destinations.get(detected_color, "NONE")
)

# Save camera images for verification
cv2.rectangle(
    frame,
    (x1, y1),
    (x2, y2),
    (0, 255, 0),
    2
)

cv2.imwrite("classification_result.jpg", frame)
cv2.imwrite("roi.jpg", roi)

# -------------------------
# ARDUINO COMMUNICATION
# -------------------------

if detected_color in ["RED", "GREEN", "BLUE"]:

    print("\nConnecting to Arduino...")

    with serial.Serial(
        PORT,
        BAUD,
        timeout=0.2
    ) as arduino:

        # Arduino Uno may reset when serial opens
        time.sleep(3)
        arduino.reset_input_buffer()

        # Check route first, without moving motors
        check_command = f"CHECK {detected_color}\n"

        print("Checking:", check_command.strip())
        arduino.write(check_command.encode("utf-8"))
        arduino.flush()

        time.sleep(0.5)

        responses = []
        while arduino.in_waiting:
            line = arduino.readline().decode(
                "utf-8", errors="replace"
            ).strip()
            if line:
                responses.append(line)
                print("Arduino:", line)

        route_ok = any(
            "Route model check OK" in line
            for line in responses
        )

        if not route_ok:
            print("Route rejected. No movement commanded.")

        else:
            # Only execute if explicitly confirmed
            answer = input(
                f"Run {detected_color} route? (yes/no): "
            ).strip().lower()

            if answer == "yes":

                # Your Arduino requires ARM first.
                arduino.write(b"ARM\n")
                arduino.flush()
                time.sleep(1)

                while arduino.in_waiting:
                    print(
                        "Arduino:",
                        arduino.readline().decode(
                            errors="replace"
                        ).strip()
                    )

                command = f"RUN {detected_color}\n"

                print("Sending:", command.strip())
                arduino.write(command.encode("utf-8"))
                arduino.flush()

                # Monitor the route until completion
                deadline = time.monotonic() + 180

                while time.monotonic() < deadline:
                    line = arduino.readline().decode(
                        "utf-8", errors="replace"
                    ).strip()

                    if not line:
                        continue

                    print("Arduino:", line)

                    if "Color route complete" in line:
                        print("Route finished.")
                        break

                    if line.startswith("ERR:"):
                        print("Arduino reported an error.")
                        break

                else:
                    print("Monitoring timed out.")
                    print("The arm may still be active.")

else:
    print("\nNo sufficiently dominant color detected.")
    print("No command sent to Arduino.")