#!/usr/bin/env python3

import cv2
import numpy as np
import mediapipe as mp
import time
from gretchen.robot import Robot
import math
from ultralytics import YOLO 

# --- global var
num = 0
sstook = False
zoom_level = 1.0

# --- YOLO load
model = YOLO('ultralytics/yolov10s.best.pt')

# --- Utility Functions

def ema(prev, new, alpha):
    return alpha * new + (1 - alpha) * prev

def l2dist(x_a, x_b, y_a, y_b):
    return math.sqrt((x_a - x_b)**2 + (y_a - y_b)**2)

def is_ok_sign(c):
    dist1 = l2dist(c["x1"], c["x2"], c["y1"], c["y2"])
    dist2 = l2dist(c["x0"], c["x3"], c["y0"], c["y3"])
    # print(dist1, dist2)
    return dist1 < 40 and dist2 > 120

def main():
    global num, sstook, zoom_level

    # gretchen cam
    robot = Robot('/dev/tty.usbserial-FT7WBH8R', 0)
    gcam = robot.camera
    robot.start()
    robot.move(0.0, 0.0)

    # laptop cam
    cap = cv2.VideoCapture(2)
    W, H = 640, 480
    # W, H = 960, 720
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)

    if not cap.isOpened():
        print("Error cam not found")
        return

    # mediapipe
    mp_face_mesh = mp.solutions.face_mesh
    mp_hands = mp.solutions.hands
    
    face_mesh = mp_face_mesh.FaceMesh(
        max_num_faces=1,
        refine_landmarks=True,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    )

    hands = mp_hands.Hands(
        max_num_hands=2,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    )

    filtered_yaw, filtered_pitch = 0.0, 0.0
    ALPHA = 0.35  
    YAW_SENSITIVITY = 2.8
    PITCH_SENSITIVITY = 3.2 
    YAW_LIMIT, PITCH_LIMIT = 0.6, 0.4
    PITCH_OFFSET = 0.08 
    last_send_time = 0.0

    try:
        while True:
            ret, frame = cap.read()
            if not ret: break
            
            frame = cv2.flip(frame, 1) 
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            face_results = face_mesh.process(rgb)
            hand_results = hands.process(rgb)

            # --- Face Synchro
            if face_results.multi_face_landmarks:
                lms = face_results.multi_face_landmarks[0].landmark
                raw_yaw = (lms[234].z - lms[454].z) * YAW_SENSITIVITY
                raw_pitch = (lms[10].z - lms[152].z) * PITCH_SENSITIVITY + PITCH_OFFSET
                target_yaw = np.clip(raw_yaw, -YAW_LIMIT, YAW_LIMIT)
                target_pitch = np.clip(raw_pitch, -PITCH_LIMIT, PITCH_LIMIT)
                filtered_yaw = ema(filtered_yaw, target_yaw, ALPHA)
                filtered_pitch = ema(filtered_pitch, target_pitch, ALPHA)

                for idx in [10, 152, 234, 454]:
                    cx, cy = int(lms[idx].x * W), int(lms[idx].y * H)
                    cv2.circle(frame, (cx, cy), 3, (0, 255, 0), -1)

            # --- Multi-hand processing
            left_hand_coords = None
            right_hand_coords = None

            if hand_results.multi_hand_landmarks:
                for idx, hand_lms in enumerate(hand_results.multi_hand_landmarks):
                    label = hand_results.multi_handedness[idx].classification[0].label
                    lm = hand_lms.landmark
                    coords = {
                        "x0": int(lm[0].x * W), "y0": int(lm[0].y * H),
                        "x1": int(lm[4].x * W), "y1": int(lm[4].y * H),
                        "x2": int(lm[8].x * W), "y2": int(lm[8].y * H),
                        "x3": int(lm[12].x * W), "y3": int(lm[12].y * H),
                        "x4": int(lm[16].x * W), "y4": int(lm[16].y * H),
                        "x5": int(lm[20].x * W), "y5": int(lm[20].y * H)
                    }
                    for i in range(21):
                        cv2.circle(frame, (int(lm[i].x * W), int(lm[i].y * H)), 2, (255, 0, 0), -1)

                    if label == "Left":
                        left_hand_coords = coords
                    else:
                        right_hand_coords = coords

            # --- gretchen camera logic
            retg, gimg, ts = gcam.getImage()
            if retg and gimg is not None:
                gview = gimg.copy()
                gh, gw = gview.shape[:2]

                # left hand ss or yolo
                if left_hand_coords:
                    c = left_hand_coords
                    
                    # ok -> yolo
                    if is_ok_sign(c):
                        cv2.putText(frame, "STATUS: IDENTIFYING...", (20, 160), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        
                        # yolo on gview
                        results = model.predict(gview, conf=0.5, verbose=False)
                        for r in results:
                            for box in r.boxes:
                                b = box.xyxy[0].cpu().numpy().astype(int)
                                cls = int(box.cls[0])
                                name = model.names[cls]
                                conf = box.conf[0]
                                
                                #detect output
                                cv2.rectangle(gview, (b[0], b[1]), (b[2], b[3]), (0, 255, 0), 2)
                                cv2.putText(gview, f"{name} {conf:.2f}", (b[0], b[1]-10), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                    # screenshot
                    total_dist = (l2dist(c["x0"], c["x1"], c["y0"], c["y1"]) + 
                                  l2dist(c["x0"], c["x2"], c["y0"], c["y2"]) + 
                                  l2dist(c["x0"], c["x3"], c["y0"], c["y3"]) + 
                                  l2dist(c["x0"], c["x4"], c["y0"], c["y4"]) + 
                                  l2dist(c["x0"], c["x5"], c["y0"], c["y5"]))
                    
                    if total_dist < 500: 
                        if not sstook:
                            cv2.imwrite(f"ss_{num}.jpg", gview)
                            print(f"System: Screenshot Saved (ss_{num}.jpg)")
                            num += 1
                            sstook = True
                    elif total_dist > 1000:
                        sstook = False
                else:
                    sstook = False
                
                # print(sstook)

                # right hand digital zoom
                if right_hand_coords:
                    c = right_hand_coords
                    if l2dist(c["x0"], c["x2"], c["y0"], c["y2"]) > 90 and \
                       l2dist(c["x0"], c["x3"], c["y0"], c["y3"]) > 100:
                        finger_gap = l2dist(c["x2"], c["x3"], c["y2"], c["y3"])
                        zoom_level = 1.0 + ((finger_gap - 40) / 60.0) 
                        zoom_level = np.clip(zoom_level, 1.0, 3.5)
                    else:
                        zoom_level = 1.0

                # digital zoom
                if zoom_level > 1.0:
                    nw, nh = int(gw / zoom_level), int(gh / zoom_level)
                    sx, sy = (gw - nw) // 2, (gh - nh) // 2
                    gview = cv2.resize(gview[sy:sy+nh, sx:sx+nw], (gw, gh))

                # text info
                cv2.putText(frame, f"Zoom: {zoom_level:.1f}x", (20, 80), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                if sstook:
                    cv2.putText(frame, "SCREENSHOT CAPTURED", (20, 120), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

                cv2.imshow("Gretchen View", gview)

            # robot move
            now = time.time()
            if now - last_send_time > 0.025:
                robot.move(float(filtered_yaw), float(filtered_pitch))
                last_send_time = now

            cv2.imshow("Controller", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\nQuit by user")
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("Quitting")

if __name__ == "__main__":
    main()