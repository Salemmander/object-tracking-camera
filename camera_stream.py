from picamera2 import Picamera2
from flask import Flask, Response
import cv2
from ultralytics import YOLO
import os
import urllib.request
from pan_tilt_control import set_servo_angle, get_angle, cleanup, PAN, TILT
from PID import PID
import time

app = Flask(__name__)
kp = 0.01
ki = 0.00005
kd = 0.1
pid_pan = PID(kp=kp, ki=ki, kd=kd)
pid_tilt = PID(kp=kp, ki=ki, kd=kd)

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

frame_counter = 0


def generate_frames():
    global frame_counter
    prev_time = time.time()
    last_annotated_frame = None

    while True:

        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time
        frame = camera.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = cv2.flip(frame, 1)

        if frame_counter % 2 == 0:
            results = model(frame, imgsz=320, conf=0.25, verbose=False)
            last_annotated_frame = results[0].plot()

            if len(results[0].boxes) > 0:
                box = results[0].boxes[0].xyxy[0].cpu().numpy()
                cx = (box[0] + box[2]) / 2
                cy = (box[1] + box[3]) / 2
                height, width = frame.shape[:2]

                error_x = cx - width / 2
                error_y = cy - height / 2

                delta_pan = pid_pan.compute(-error_x, dt)
                delta_tilt = pid_tilt.compute(error_y, dt)

                current_pan = get_angle(PAN)
                current_tilt = get_angle(TILT)
                set_servo_angle(PAN, current_pan + delta_pan)
                set_servo_angle(TILT, current_tilt + delta_tilt)

        frame_counter += 1

        output_frame = (
            last_annotated_frame if last_annotated_frame is not None else frame
        )

        ret, buffer = cv2.imencode(".JPG", output_frame)
        frame_bytes = buffer.tobytes()
        yield (
            b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n"
        )


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


if __name__ == "__main__":
    try:
        set_servo_angle(PAN, 0)
        set_servo_angle(TILT, 0)
        app.run(host="0.0.0.0", port=5000)
    except KeyboardInterrupt:
        cleanup()
    except Exception as e:
        print(f"Error: {e}")
        cleanup()
    finally:
        cleanup()
