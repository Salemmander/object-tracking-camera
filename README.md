# Object Tracking Camera  

**by @Salemmmander**

![demo](demo.gif)  
*Real-time face tracking on a Raspberry Pi 5 — YOLOv11 + PID-controlled pan/tilt servos.

---

## What It Does

- **Detects faces** using **YOLOv11n-face** (runs at ~25–30 FPS on Pi 5)  
- **Tracks smoothly** with a **custom PID controller** (compiled C → `PID.so`)  
- **Follows you** using **PCA9685 + two SG90 servos** (pan: 0–180°, tilt: 0–180°)  
- **Streams live** via Flask (`http://pi-ip:5000`)  

---

## Hardware

- [Raspberry Pi 5](https://www.raspberrypi.com/products/raspberry-pi-5/)
- [Pi Camera Module 3](https://www.raspberrypi.com/products/camera-module-3/)
- [Waveform Pan-Tilt HAT](https://www.waveshare.com/pan-tilt-hat.htm)
