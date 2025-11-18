# Driver-Sleepy-alert-system
# Sleepy Driver — README

**Sleepy Driver** — Real-time driver drowsiness & yawning detection (Python + Mediapipe) with Arduino alerts (buzzer, vibration) and motor control.
This README explains **how to set up**, **how to run** (Python + Arduino), and **troubleshooting tips** so you can upload this project to GitHub and anyone can reproduce it.

---

## Repo structure (recommended)

```
sleepy-driver/
├─ arduino/
│  └─ sleepy_driver.ino            # Arduino sketch
├─ python/
│  ├─ main.py                      # Core detection script (EAR + MAR + MP3 alert)
│  ├─ gui.py                       # Optional GUI version (Tkinter)
│  ├─ requirements.txt             # Python dependencies
│  └─ alert_voice.mp3              # Put your voice MP3 here
├─ docs/
│  └─ circuit_diagram.png
├─ README.md
└─ LICENSE
```

---

## 1 — Hardware list

* Arduino UNO (or compatible)
* L293D motor driver (or TB6612FNG recommended)
* DC motor
* Vibration motor + NPN transistor (2N2222/BC547) + 1kΩ resistor
* Buzzer
* Breadboard, jumper wires
* USB cable (Arduino ↔ laptop)
* Motor power supply (recommended: USB power bank 5V≥2A or 6×AA NiMH pack) — **do not** power motor from laptop USB directly
* Electrolytic capacitor (1000µF, 16V) across motor supply
* Multimeter (recommended)

---

## 2 — Arduino: code & upload

### File

`arduino/sleepy_driver.ino` — (example behavior)

* Pins:

  * Buzzer → D8
  * Vibration (via NPN) → D9
  * L293D IN1 → D10
  * L293D IN2 → D11
  * L293D EN → D6 (PWM)

### Steps to upload

1. Open Arduino IDE.
2. Connect Arduino to your computer with USB.
3. Open `arduino/sleepy_driver.ino`.
4. Tools → Board → **Arduino Uno**.
5. Tools → Port → select the Arduino COM/CU port (Windows `COMx`, Mac `/dev/cu.usbmodemxxxxx`).
6. Click **Upload**.
7. Open **Serial Monitor** (9600 baud) to debug (close it while Python will use the port).

### Behavior

* Arduino listens on Serial at 9600 baud for `ALERT` or `NORMAL`.
* `ALERT` -> buzzer ON, vibration ON, braking/ramp-down motor.
* `NORMAL` -> buzzer OFF, vibration OFF, motor restarts.

---

## 3 — Python: environment & dependencies

Two options: **virtualenv** (venv) or **conda**. Use Python **3.9** for best Mediapipe compatibility.

### Using venv (cross-platform)

```bash
# from repo root
cd python
python3 -m venv venv
# activate:
# mac / linux:
source venv/bin/activate
# windows:
venv\Scripts\activate

pip install -r requirements.txt
```

### Using conda

```bash
conda create -n sleepy_driver39 python=3.9
conda activate sleepy_driver39
pip install -r requirements.txt
```

### requirements.txt (example)

```
opencv-python
mediapipe
numpy
pyserial
pygame
pillow         # for GUI
tk             # (on some systems)
```

> Note: `playsound` has macOS issues. This repo uses `pygame` for playing MP3 reliably across platforms.

---

## 4 — Python script settings

### Important constants (edit at top of `main.py` or `gui.py`)

```python
ARDUINO_PORT = "/dev/cu.usbmodem11301"  # mac example
# OR Windows: "COM3"
BAUD_RATE = 9600
ALERT_AUDIO = "alert_voice.mp3"         # put your mp3 here
DROWSY_TIME = 3                         # seconds eyes closed before alert
MAR_YAWN_THRESHOLD = 0.65               # mouth aspect ratio threshold for yawning
```

### How serial communication works

* Python opens serial to `ARDUINO_PORT` at 9600 baud.
* When drowsiness detected: `arduino.write(b'ALERT\n')`
* When eyes normal: `arduino.write(b'NORMAL\n')`

**Only one program can use the serial port**. Close Arduino Serial Monitor before running Python.

---

## 5 — Run (no GUI)

From `python/` venv activated, run:

```bash
python main.py
```

* The script will:

  * Start webcam
  * Auto-calibrate (first ~50 frames) — keep eyes open
  * Display EAR & MAR on-screen
  * Play `alert_voice.mp3` with pygame when ALERT
  * Send `ALERT`/`NORMAL` to Arduino when available

To quit: press `ESC` in the video window.

---

## 6 — Run (GUI version)

GUI combines detection + controls + calibration:

```bash
python gui.py
```

* Use **Start** to begin detection.
* Click **Calibrate** while eyes open to re-run threshold calibration.
* Use sliders to adjust EAR/MAR thresholds or drowsy time.
* **Test Alert** button sends manual `ALERT` to Arduino for testing.

---



## 7 — Troubleshooting & tips

### Serial port busy error on mac (`Resource busy`)

* Close Arduino Serial Monitor & Serial Plotter.
* Reconnect USB cable.
* Check `lsof | grep usbmodem` and kill any process using the port.

### MP3 playback fails on mac with playsound

* Use `pygame` instead; make sure `pygame.mixer.init()` before playing.
* Example non-blocking player:

  ```python
  import pygame, threading
  pygame.mixer.init()
  def play_alert(file):
      threading.Thread(target=lambda: (pygame.mixer.music.load(file), pygame.mixer.music.play()), daemon=True).start()
  ```

### Mediapipe issues / installation notes

* Use Python 3.9.
* On Mac ARM (M1/M2) use `conda` or the correct wheel for mediapipe; if installation fails, try conda-forge.

### Motor slow / stops after a while

* Replace 9V PP3 battery with a better power source:

  * 6×AA NiMH pack (~7.2V) or a USB power bank (5V ≥2A) (if motor accepts 5V)
* Add electrolytic capacitor (1000µF) + thicker wires
* Consider switching L293D → TB6612FNG for lower voltage drop
* Ensure common ground: Arduino GND ↔ motor power negative

### If Arduino not found

* On Windows: check Device Manager → Ports (COM & LPT)
* On Mac: `ls /dev/cu.*` → pick `/dev/cu.usbmodem*` or `/dev/cu.usbserial*`
* Update `ARDUINO_PORT` accordingly

---

## 8 — Safety & best practices

* Always disconnect power when re-wiring.
* Do not power motor from laptop USB port directly.
* Use a current-capable power source for motors and tie grounds.
* Use a multimeter to verify voltages before connecting motors.

---

## 10 — Example usage flow (demo script)

1. Upload Arduino sketch to Arduino via Arduino IDE.
2. Build hardware and keep Arduino connected to laptop via USB for serial.
3. Connect motor power from a USB power bank (not Arduino 5V pin). Tie grounds.
4. Ensure `alert_voice.mp3` is present in `python/` folder.
5. Activate Python venv and `pip install -r requirements.txt`.
6. Run `python gui.py` or `python main.py`.
7. Calibrate (if GUI press Calibrate, or auto-calibration runs in `main.py`).
8. Test by manually sending `ALERT` via Serial Monitor (Arduino) OR use yaw/eye closure to trigger.

---

## 11 — Contribution & License

* Please add improvements (IR camera, head pose detection, cloud alerts).
* Suggested license: **MIT** — create `LICENSE` file.

---

## 12 — References & Further Reading

* OpenCV — [https://opencv.org](https://opencv.org)
* Mediapipe — [https://google.github.io/mediapipe](https://google.github.io/mediapipe)
* PySerial docs — [https://pyserial.readthedocs.io](https://pyserial.readthedocs.io)
* TB6612FNG driver (recommended) — Adafruit / Pololu docs

---


