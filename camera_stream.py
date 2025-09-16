from picamera2 import Picamera2
from flask import Flask, Response
import cv2
from ultralytics import YOLO
import os
import urllib.request
import ctypes
from controller import PanTiltController
import time

app = Flask(__name__)


# region CTYPES

lib = ctypes.CDLL("./PID.so")

lib.PID_new.argtypes = [ctypes.c_double, ctypes.c_double, ctypes.c_double]
lib.PID_new.restype = ctypes.c_void_p
lib.PID_reset.argtypes = [ctypes.c_void_p]
lib.PID_delete.argtypes = [ctypes.c_void_p]
lib.PID_compute.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_double]
lib.PID_compute.restype = ctypes.c_double

# endregion

# region Camera and Model Setup

H_FOV = 102.0  # Horizontal Field of View for Pi Camera v3
V_FOV = 74.0  # Vertical Field of View for Pi Camera v3

camera = Picamera2()

camera_config = camera.create_video_configuration()
camera.configure(camera_config)
camera.start()

model_path = "yolov11n-face.pt"
model_url = (
    "https://github.com/akanametov/yolo-face/releases/download/v0.0.0/yolov11n-face.pt"
)
if not os.path.exists(model_path):
    print(f"Downloading {model_path} from {model_url}...")
    try:
        urllib.request.urlretrieve(model_url, model_path)
        print(f"Downloaded {model_path} successfully.")
    except Exception as e:
        print(f"Error downloading {model_path}: {e}")
        exit(1)

model = YOLO("yolov11n-face.pt")

# endregion

# region Pan-Tilt_controller and PID Setup

controller = PanTiltController()
current_pan = 90
current_tilt = 90
controller.set_servo_angle(controller.PAN, current_pan)
controller.set_servo_angle(controller.TILT, current_tilt)

pid_pan = lib.PID_new(0.4, 0.0, 0.1)
pid_tilt = lib.PID_new(0.4, 0.0, 0.1)

last_time = time.time()


# endregion
def generate_frames():
    global current_pan, current_tilt, last_time
    while True:
        frame = camera.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = cv2.flip(frame, 1)

        results = model(frame, imgsz=320, conf=0.25, verbose=False)

        frame_yc, frame_xc = frame.shape[0] / 2, frame.shape[1] / 2
        annotated_frame = frame
        face_detected = False

        try:
            face_xc, face_yc = results[0].boxes.xywh[0].tolist()[:2]
            annotated_frame = results[0].plot()
            cv2.circle(
                annotated_frame, (int(face_xc), int(face_yc)), 3, (0, 0, 255), -1
            )
            face_detected = True
        except IndexError:
            # Handle no detections
            pass
        cv2.circle(annotated_frame, (int(frame_xc), int(frame_yc)), 3, (0, 0, 255), -1)

        if face_detected:
            pixel_error_x = face_xc - frame_xc
            pixel_error_y = face_yc - frame_yc
            pixel_per_degree_x = frame.shape[1] / H_FOV
            pixel__per_degree_y = frame.shape[0] / V_FOV
            angle_error_x = pixel_error_x / pixel_per_degree_x
            angle_error_y = pixel_error_y / pixel__per_degree_y
            # print(f"Angle Error X: {angle_error_x}, Angle Error Y: {angle_error_y}")

            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time
            if dt <= 0:
                dt = 1e-6

            delta_pan = lib.PID_compute(pid_pan, angle_error_x, dt)
            delta_tilt = lib.PID_compute(pid_tilt, angle_error_y, dt)

            current_pan = max(0, min(180, current_pan + delta_pan))
            current_tilt = max(0, min(180, current_tilt + delta_tilt))

            controller.set_servo_angle(controller.PAN, current_pan)
            controller.set_servo_angle(controller.TILT, current_tilt)
            cv2.putText(
                annotated_frame,
                f"Pan: {current_pan:.2f}, Tilt: {current_tilt:.2f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
        else:
            lib.PID_reset(pid_pan)
            lib.PID_reset(pid_tilt)

        ret, buffer = cv2.imencode(".JPG", annotated_frame)
        frame_bytes = buffer.tobytes()
        yield (
            b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n"
        )


# region Video Feed Route


@app.route("/video_feed")
def video_feed():
    return Response(
        generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/")
def index():
    return """
    <html>
    <body>
        <h1>Raspberry Pi Camera Stream</h1>
        <img src="/video_feed">
    </body>
    </html>
    """


# endregion

if __name__ == "__main__":
    try:
        app.run(host="0.0.0.0", port=5000)
    except KeyboardInterrupt:
        print("Shutting down...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        controller.cleanup()
