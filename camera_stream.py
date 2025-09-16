from picamera2 import Picamera2
from flask import Flask, Response
import cv2
from ultralytics import YOLO
import os
import urllib.request

from deep_sort_realtime.deepsort_tracker import DeepSort

app = Flask(__name__)

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

tracker = DeepSort(max_age=30)


def generate_frames():

    while True:
        frame = camera.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = cv2.flip(frame, 1)

        results = model(frame, imgsz=320, conf=0.25, verbose=False)

        frame_yc, frame_xc = frame.shape[0] / 2, frame.shape[1] / 2

        try:
            face_xc, face_yc = results[0].boxes.xywh[0].tolist()[:2]
            annotated_frame = results[0].plot()
            cv2.circle(
                annotated_frame, (int(face_xc), int(face_yc)), 3, (0, 0, 255), -1
            )
        except IndexError:
            annotated_frame = frame
        cv2.circle(annotated_frame, (int(frame_xc), int(frame_yc)), 3, (0, 0, 255), -1)

        ret, buffer = cv2.imencode(".JPG", annotated_frame)
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
        app.run(host="0.0.0.0", port=5000)
    except KeyboardInterrupt:
        print("oops")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("oops")
