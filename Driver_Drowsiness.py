import cv2
import mediapipe as mp
import numpy as np
import serial
import time
import threading
from playsound import playsound

# ========= MP3 Alert Function =========
import pygame
import threading

pygame.mixer.init()

def play_alert():
    def _play():
        pygame.mixer.music.load("agg1.mp3")   # your audio file
        pygame.mixer.music.play()
    threading.Thread(target=_play, daemon=True).start()


# ========= Arduino Setup =========
ARDUINO_PORT = "/dev/cu.usbmodem11301"   # update if needed
BAUD_RATE = 9600

try:
    arduino = serial.Serial(ARDUINO_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)
    print("✅ Arduino connected.")
except:
    arduino = None
    print("⚠️ Arduino not connected. Running without hardware.")


# ========= Mediapipe Setup =========
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(refine_landmarks=True, max_num_faces=1)

LEFT_EYE = [33, 160, 158, 133, 153, 144]
RIGHT_EYE = [362, 385, 387, 263, 373, 380]
MOUTH = [13, 14, 78, 308, 82, 312]


def dist(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))


def EAR(landmarks, eye, w, h):
    pts = np.array([(int(landmarks[p].x * w), int(landmarks[p].y * h)) for p in eye])
    A = dist(pts[1], pts[5])
    B = dist(pts[2], pts[4])
    C = dist(pts[0], pts[3])
    return (A + B) / (2.0 * C), pts


def MAR(landmarks, w, h):
    pts = np.array([(int(landmarks[p].x * w), int(landmarks[p].y * h)) for p in MOUTH])
    vertical = dist(pts[0], pts[1])
    horizontal = dist(pts[2], pts[3])
    return vertical / horizontal, pts


# ========= Calibration Variables =========
open_ear_avg = 0.0
calibration_frames = 50
cal_count = 0
calibrating = True

# ========= Detection Settings =========
DROWSY_TIME = 3     # seconds eyes closed before alert
alert_sent = False
start_time = None

# ========= Video Setup =========
cap = cv2.VideoCapture(0)
cap.set(3, 960)
cap.set(4, 720)

print("📷 Keep eyes OPEN for calibration...")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.flip(frame, 1)
    h, w, _ = frame.shape

    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    result = face_mesh.process(rgb)

    if result.multi_face_landmarks:
        lm = result.multi_face_landmarks[0].landmark

        left_ear, left_pts = EAR(lm, LEFT_EYE, w, h)
        right_ear, right_pts = EAR(lm, RIGHT_EYE, w, h)
        ear = (left_ear + right_ear) / 2.0

        mar, mouth_pts = MAR(lm, w, h)

        # draw points
        for (x, y) in np.concatenate((left_pts, right_pts, mouth_pts)):
            cv2.circle(frame, (x, y), 2, (0, 255, 0), -1)

        # -------- Calibration Phase --------
        if calibrating:
            open_ear_avg += ear
            cal_count += 1
            cv2.putText(frame, "CALIBRATING... Keep eyes OPEN", (100, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

            if cal_count >= calibration_frames:
                open_ear_avg /= calibration_frames
                EAR_THRESHOLD = open_ear_avg * 0.7
                calibrating = False

                print(f"✅ Calibration Complete!")
                print(f"Open EAR Avg = {open_ear_avg:.3f}")
                print(f"EAR Threshold = {EAR_THRESHOLD:.3f}")

        # -------- Detection Phase --------
        else:

            # Show EAR & MAR
            cv2.putText(frame, f"EAR: {ear:.3f}", (30, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

            cv2.putText(frame, f"MAR: {mar:.3f}", (30, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)

            # ========== YAWNING DETECTION ==========
            if mar > 0.65:  
                cv2.putText(frame, "Yawning Detected", (200, 120),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,255), 3)

                # Cancel alert if yawning
                start_time = None
                if alert_sent:
                    if arduino: arduino.write(b'NORMAL\n')
                    alert_sent = False

                cv2.imshow("Driver Monitor", frame)
                if cv2.waitKey(1) & 0xFF == 27:
                    break
                continue

            # ========= Eye Closure Detection ==========
            if ear < EAR_THRESHOLD:
                if start_time is None:
                    start_time = time.time()

                elapsed = time.time() - start_time

                if elapsed > DROWSY_TIME and not alert_sent:
                    print("⚠️ Drowsiness Detected!")

                    if arduino:
                        arduino.write(b'ALERT\n')

                    play_alert()  # your voice MP3
                    alert_sent = True

            else:
                start_time = None
                if alert_sent:
                    print("✅ Eyes open — NORMAL")
                    if arduino:
                        arduino.write(b'NORMAL\n')
                    alert_sent = False

    else:
        cv2.putText(frame, "No face detected", (30, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

    cv2.imshow("Driver Monitor", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
if arduino:
    arduino.close()
print("🛑 System stopped.")
