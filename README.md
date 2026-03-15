# Track and Surveil

## 📌 Overview
This project implements real-time face and hand tracking using MediaPipe
and controls Gretchen robot head motors and camera accordingly.

User's head is synchronized to gretchen, when user turns, gretchen turns
User's right hand is used as a zoom in/out cursor, the distance between index and middle finger decides the zoom level
User's left hand is used to signal 'taking screenshots' and 'start face detection'.
    Clenching a fist indicates 'take screenshot'
    Opening the fist indicates 'to screenshot-ready mode'
    'OK sign' indicates use face detection(detects while the ok sign is visible)

## Features
    👤 User head movement is mirrored by Gretchen's head motors
    🤚 Right hand controls camera zoom - Distance between index and middle finger determines zoom level
    ✋ Left hand gesture controls:
        ✊ Fist → Take screenshot
        ✋ Open hand → Screenshot-ready mode
        👌 OK sign → Enable real-time face detection (detects while the ok sign is visible)

## System Requirements
This project requires two separate Python environments:

### Environment	Purpose
Python 3.10	Vision system (MediaPipe, YOLO, OpenCV)
Python 3.12	Robot control (Dynamixel SDK, Gretchen SDK)

### Main Dependencies
OpenCV
MediaPipe 0.10.14
Ultralytics (YOLO)
dlib
NumPy
PySerial
Gretchen SDK
Dynamixel SDK

## 🛠 Installation
---- INSIDE the project dir ----

1. Vision Environment (Python 3.10)
Used for:
    Face tracking
    Hand tracking
    Gesture recognition
    YOLO detection
```
python3.10 -m venv vision_env
source vision_env/bin/activate

pip install numpy opencv-python opencv-contrib-python
pip install mediapipe==0.10.14
pip install ultralytics dlib imutils
```

Deactivate when done:
```
deactivate
```

2. Robot Environment (Python 3.12)
Used for:
    Motor control
    Serial communication
    Gretchen SDK

```
python3.12 -m venv robot_env
source robot_env/bin/activate
```

Install required Python packages:
```
pip install numpy pyserial setuptools opencv-python
```

## Install Dynamixel SDK

Navigate to:
```
gretchen/course_material/lib/DynamixelSDK/python
```

Then run:
```
python setup.py install
```

## Install Gretchen SDK

Navigate to:
```
gretchen/course_material/lib
```

Then run:
```
python setup.py install
```

Deactivate environment after installation:
```
deactivate
```

# extra libraries: pyqt5, lxml for labelImg, albumentations opencv-python pillow for augmentation
``` bash
pip install pyqt5 lxml albumentations opencv-python pillow
```

## Testing
Please test the project with provided pics in the test pics folder.
Yolo is custom trained on our faces.
