import cv2
import mediapipe as mp
import serial
import time
import sys
import os
import subprocess
import shutil
from datetime import datetime
import signal
import csv

# Camera/Control parameters
CAMERA_ID = 0
FRAME_WIDTH = 800
FRAME_HEIGHT = 600

# Serial parameters
BAUD_RATE = 9600
AUTO_UPLOAD = True 

# Control tuning
FORWARD_ON = True
FORWARD_MS = 120          
FORWARD_SEND_INTERVAL = 0.35      
DEADZONE_X = 0.12           
TURN_MS = 150            
TURN_INTERVAL = 0.12     
SMOOTHING = 0.9         
INVERT_TURN = True      
MIN_TURN_MS = 90         
MAX_TURN_MS = 50         
TURN_SEND_INTERVAL = 0.35  

# Logging
LOG_DIR = "logs"

def get_serial_port():
    """
    Try to auto-detect Arduino/Nano/CH340/CP210x/FTDI COM port.
    Returns device path like 'COM3' or None if not found.
    """
    try:
        from serial.tools import list_ports
        ports = list(list_ports.comports())
        candidates = []
        known_pairs = {
            (0x2341, 0x0043),  # Arduino
            (0x1A86, 0x7523),  # CH340
            (0x10C4, 0xEA60),  # CP210x
            (0x0403, 0x6001),  # FTDI
        }
        for p in ports:
            desc_lower = (p.description or "").lower()
            vidpid = (p.vid, p.pid)
            if any(k in desc_lower for k in ["arduino", "ch340", "usb-serial", "cp210", "ftdi", "nano"]) or (vidpid in known_pairs):
                candidates.append(p.device)
        if candidates:
            print(f"[INFO] Detected serial candidates: {candidates}")
            return candidates[0]
        if ports:
            print(f"[INFO] Fallback to first COM: {ports[0].device}")
            return ports[0].device
    except Exception as e:
        print(f"[WARN] Serial port detection error: {e}")
    return None

# Embedded Arduino Nano sketch (nano_receiver.ino) matching 'l/r/s' protocol at 9600 baud
NANO_RECEIVER_INO = r'''
#include <Servo.h>
Servo servoRight;
Servo servoLeft;
byte CMDstart;

int RIGHT_STOP_US = 1500;
int LEFT_STOP_US  = 1500;

int RIGHT_FWD_US = 1300;
int LEFT_FWD_US  = 1700;

void driveFor(int rUs, int lUs, int tMs) {
  digitalWrite(13, HIGH);
  servoRight.writeMicroseconds(rUs);
  servoLeft.writeMicroseconds(lUs);
  delay(tMs);
  // auto-stop after duration
  servoRight.writeMicroseconds(RIGHT_STOP_US);
  servoLeft.writeMicroseconds(LEFT_STOP_US);
  digitalWrite(13, LOW);
}

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(200);
  servoRight.attach(5);
  servoLeft.attach(6);
  pinMode(13, OUTPUT);
  // Set neutral to stop both wheels at startup
  servoRight.writeMicroseconds(RIGHT_STOP_US);
  servoLeft.writeMicroseconds(LEFT_STOP_US);
  delay(50);
}

void loop() {
  while (Serial.available() > 0) {
    delay(1);
    CMDstart = Serial.read();

    if (CMDstart == 'f') { // forward
      int t = Serial.parseInt();
      driveFor(RIGHT_FWD_US, LEFT_FWD_US, t);
    }
    if (CMDstart == 'l') { // turn left
      int t = Serial.parseInt();
      driveFor(1700, 1700, t);
    }
    if (CMDstart == 'r') { // turn right
      int t = Serial.parseInt();
      driveFor(1300, 1300, t);
    }
    if (CMDstart == 'b') { // backward
      int t = Serial.parseInt();
      driveFor(1700, 1300, t);
    }
    if (CMDstart == 's') { // stop
      int t = Serial.parseInt();
      servoRight.writeMicroseconds(RIGHT_STOP_US);
      servoLeft.writeMicroseconds(LEFT_STOP_US);
      delay(t);
    }
  }
}
'''

def has_arduino_cli():
    return shutil.which("arduino-cli") is not None

def create_nano_receiver_sketch():
    if not os.path.exists("nano_receiver"):
        os.makedirs("nano_receiver")
    with open(os.path.join("nano_receiver", "nano_receiver.ino"), "w", encoding="utf-8") as f:
        f.write(NANO_RECEIVER_INO)

def upload_to_nano(port):
    try:
        # Ensure Servo library is installed
        try:
            subprocess.run(["arduino-cli", "lib", "install", "Servo"], check=True)
        except subprocess.CalledProcessError as e:
            print("[upload] Failed to install Servo library:", e)
            return False

        # Prepare sketch and compile
        create_nano_receiver_sketch()
        subprocess.run(["arduino-cli", "compile", "--fqbn", "arduino:avr:nano:cpu=atmega328old", "nano_receiver"], check=True)

        # Release COM port if open elsewhere
        try:
            s = serial.Serial(port)
            s.close()
        except Exception:
            pass
        time.sleep(2)

        # Upload
        subprocess.run(["arduino-cli", "upload", "-p", port, "--fqbn", "arduino:avr:nano:cpu=atmega328old", "nano_receiver"], check=True)
        print("[upload] Arduino sketch uploaded.")
        time.sleep(2)  # wait for Nano reset
        return True
    except subprocess.CalledProcessError as e:
        print(f"[upload] Upload failed: {e}")
        return False

def init_camera():
    """Initialize and configure the camera."""
    print(f"[INFO] Opening camera {CAMERA_ID}")
    cap = cv2.VideoCapture(CAMERA_ID)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    return cap

def init_face_detection():
    """Initialize MediaPipe face detection."""
    print("[INFO] Initializing Face Detection")
    mp_face = mp.solutions.face_detection
    face_detection = mp_face.FaceDetection(
        model_selection=1,
        min_detection_confidence=0.3
    )
    return face_detection

def init_arduino():
    port = "COM3"   
    print(f"[INFO] Connecting to Arduino on {port}")
    try:
        arduino = serial.Serial(port, BAUD_RATE, timeout=1)
        time.sleep(2)
        return arduino
    except Exception as e:
        print(f"[ERROR] Could not connect to Arduino: {e}")
        return None

def smooth(prev, curr, alpha=SMOOTHING):
    """Exponential moving average smoothing."""
    if prev is None:
        return curr
    return prev * alpha + curr * (1.0 - alpha)

def process_frame(frame, face_detection):
    """Detect first face, draw bbox and center; return normalized center x (0..1) or None."""
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = face_detection.process(frame_rgb)

    cx_norm = None
    if results.detections:
        # Use the first detected face
        detection = results.detections[0]
        bbox = detection.location_data.relative_bounding_box
        ih, iw, _ = frame.shape

        # Calculate face center (pixels and normalized)
        center_x_px = int((bbox.xmin + bbox.width / 2) * iw)
        center_y_px = int((bbox.ymin + bbox.height / 2) * ih)
        cx_norm = (bbox.xmin + bbox.width / 2)

        # Draw bbox and center point
        x1 = int(bbox.xmin * iw)
        y1 = int(bbox.ymin * ih)
        x2 = int((bbox.xmin + bbox.width) * iw)
        y2 = int((bbox.ymin + bbox.height) * ih)

        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(frame, (center_x_px, center_y_px), 5, (0, 0, 255), -1)

    # Draw crosshair at image center
    cvx, cvy = FRAME_WIDTH // 2, FRAME_HEIGHT // 2
    cv2.line(frame, (cvx - 20, cvy), (cvx + 20, cvy), (0, 255, 0), 2)
    cv2.line(frame, (cvx, cvy - 20), (cvx, cvy + 20), (0, 255, 0), 2)

    return frame, cx_norm

def send_control_command(arduino, command, duration_ms):
    """Send a command string '<cmd><duration>\\n' and ensure serial buffer is cleared so only latest command is processed."""
    cmd_str = f"{command}{duration_ms}"
    line = cmd_str + "\n"
    try:
        if arduino:
            # Clear input/output buffers to drop stale data and send only the latest command
            try:
                arduino.reset_input_buffer()
                arduino.reset_output_buffer()
            except Exception:
                pass

            arduino.write(line.encode("utf-8"))
            # Ensure data is pushed out immediately
            try:
                arduino.flush()
            except Exception:
                pass

            # Optionally drain any immediate echo/response to keep buffer clean
            try:
                if arduino.in_waiting:
                    arduino.read(arduino.in_waiting)
            except Exception:
                pass

        print(f"[SEND] {cmd_str}")
    except Exception as e:
        print(f"[WARN] Serial send error: {e}")

def main():
    """Main function for face-following control."""
    cap = init_camera()
    if not cap or not cap.isOpened():
        print("[ERROR] Could not open camera")
        return

    face_detection = init_face_detection()

    # Optional: compile and upload embedded Arduino sketch before connecting
    port_hint = get_serial_port()
    if AUTO_UPLOAD and has_arduino_cli() and port_hint:
        print(f"[INFO] Uploading embedded Arduino sketch to {port_hint} ...")
        ok = upload_to_nano(port_hint)
        if not ok:
            print("[WARN] Upload failed; proceeding without reflash.")
    elif AUTO_UPLOAD and not has_arduino_cli():
        print("[WARN] arduino-cli not found; skip auto upload.")

    arduino = init_arduino()

    # Setup logging file
    if not os.path.exists(LOG_DIR):
        os.makedirs(LOG_DIR)
    log_path = os.path.join(LOG_DIR, f"face_follow_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
    log_f = open(log_path, "w", encoding="utf-8", newline="")
    log_writer = csv.writer(log_f)
    log_writer.writerow(["timestamp", "cx_norm", "x_offset", "cmd", "duration_ms"])

    prev_cx = None
    last_send_time = 0.0
    last_cmd = "s"

    print("[INFO] Running (Ctrl+C to quit)")
    running = True

    def _handle_sigint(sig, frame):
        nonlocal running
        running = False
        print("\n[INFO] SIGINT received, exiting...")

    signal.signal(signal.SIGINT, _handle_sigint)

    try:
        while running:
            ret, frame = cap.read()
            if not ret:
                print("[ERROR] Camera read failed")
                break

            frame, cx_norm = process_frame(frame, face_detection)

            if cx_norm is not None:
                # Smooth the center x (normalized 0..1) and compute offset from center 0.5
                cx_smooth = smooth(prev_cx, cx_norm, SMOOTHING)
                prev_cx = cx_smooth
                x_offset = cx_smooth - 0.5

                # Decide command (face left -> turn left, face right -> turn right)
                if x_offset < -DEADZONE_X:
                    cmd_raw = "l"
                elif x_offset > DEADZONE_X:
                    cmd_raw = "r"
                else:
                    cmd_raw = "f" if FORWARD_ON else "s"

                # Optional inversion to match physical wiring/orientation
                cmd = cmd_raw
                if INVERT_TURN and cmd_raw in ("l", "r"):
                    cmd = "l" if cmd_raw == "r" else "r"

                # Fixed duration to ensure minimal steering per pulse
                if cmd == "s":
                    duration_ms = 0
                elif cmd == "f":
                    duration_ms = FORWARD_MS
                else:
                    duration_ms = MIN_TURN_MS

                # Throttle command sending
                now = time.time()
                if cmd in ("l", "r"):
                    interval = TURN_SEND_INTERVAL
                elif cmd == "f":
                    interval = FORWARD_SEND_INTERVAL
                else:
                    interval = TURN_INTERVAL
                if (now - last_send_time) >= interval and (cmd in ("l", "r") or cmd != last_cmd):
                    send_control_command(arduino, cmd, duration_ms)
                    last_send_time = now
                    last_cmd = cmd

                # Log face position and command
                try:
                    log_writer.writerow([datetime.now().isoformat(), f"{cx_smooth:.6f}", f"{x_offset:.6f}", cmd, duration_ms])
                    log_f.flush()
                except Exception:
                    pass
                # Overlay debug info
                cv2.putText(frame, f"x_offset={x_offset:.3f} cmd={cmd}",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            else:
                # No face -> stop once, throttled
                now = time.time()
                cmd = "s"
                if cmd != last_cmd and (now - last_send_time) >= TURN_INTERVAL:
                    send_control_command(arduino, "s", 0)
                    last_send_time = now
                    last_cmd = cmd
                # Log no-face state and stop command
                try:
                    log_writer.writerow([datetime.now().isoformat(), None, None, "s", 0])
                    log_f.flush()
                except Exception:
                    pass
                cv2.putText(frame, "No Face",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Display
            cv2.imshow("Face Follow", frame)
            # No keyboard exit; use Ctrl+C (SIGINT) to quit
            cv2.waitKey(1)

    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        if arduino:
            try:
                arduino.close()
            except:
                pass
        try:
            if 'log_f' in locals():
                log_f.flush()
                log_f.close()
        except Exception:
            pass
        print("[INFO] Program ended")

if __name__ == "__main__":
    main()
