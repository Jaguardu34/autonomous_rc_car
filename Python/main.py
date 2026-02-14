import cv2
from flask import Flask, render_template, jsonify, Response, request
import control
import threading
import time
import math
import numpy as np
from ultralytics import YOLO
from collections import deque


#Load Yolo Model
model = YOLO("Python/yolo_models/yolo11m-road-seg.pt")
model.to("cuda")

#Variables
battery = 0
gyro_x = 0
gyro_y = 0
gyro_z = 0
accel_x = 0
accel_y = 0
accel_z = 0
temp = 0
max_battery = 16.8
min_battery = 13.0
battery_percent = 0
action_active = False
SILENCE_TIMEOUT = 0.5 # secondes
vitesse = 15
sensibilite = 90.0
distance_front = 0.0
last_received_time=0.0

app = Flask(__name__)



control.init_serial();


def get_data():
    global battery, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, temp, battery_percent, distance_front, last_received_time
    
    serial = control.read_serial()
    
    now = time.time() 
    
    if serial == None :
        return

    
    x = float(serial[2])
    y = float(serial[1])
    z = float(serial[3])
    
    gyro_x_offset = 18

    battery = float(serial[0])
    accel_x = float(serial[4])
    accel_y = float(serial[5])
    accel_z = float(serial[6])
    temp = serial[7]
    distance_front = float(serial[8])
    
    temp = round(float(temp))
    
    #conversion en degrés  
    gyro_x = math.atan2(y, math.sqrt(x * x + z * z)) * 180 / math.pi
    gyro_y = math.atan2(-x, math.sqrt(y * y + z * z)) * 180 / math.pi
    gyro_z = math.atan2(z, math.sqrt(x * x + y * y)) * 180 / math.pi
    
    if (gyro_x <= 0):
        gyro_x = math.floor(gyro_x)
    else:
        gyro_x = math.ceil(gyro_x)
    if (gyro_y <= 0):
        gyro_y = math.floor(gyro_y)
    else:
        gyro_y = math.ceil(gyro_y)
    if (gyro_z <= 0):
        gyro_z = math.floor(gyro_z)
    else:
        gyro_z = math.ceil(gyro_z)
        
    gyro_y += gyro_x_offset
    gyro_x = -gyro_x
    
    battery_percent = round(((float(battery) - min_battery) / (max_battery - min_battery)) * 100)   
    if battery_percent > 100:
        battery_percent = 100
    elif battery_percent < 0:
        battery_percent = 0
        

def start_active():
    global action_active, distance_front
    if distance_front < 45 and distance_front != 0:
        action_active = False
    else:
        action_active = True
def main_loop() :
    print("Boucle principaled démarrée")
    global direction, action_active; distance_front, accel_x, accel_y
    first_time_backward = True
    while True:
        get_data()
 
        if action_active:
            if distance_front < 40 and distance_front != 0:
                if first_time_backward:
                    control.move(60, "b")
                    time.sleep(0.5)
                    first_time_backward = False
                    action_active = False
                
            else:
                first_time_backward = True
                control.move(vitesse)
                if direction < 0:
                    control.left(-direction)
                elif direction > 0:
                    control.right(direction)
                else:
                    control.center()
                
        else:
            control.stop()
        time.sleep(0.05)

            
                

#Video Init
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    raise RuntimeError("Impossible d’ouvrir la caméra")

# Variables globales (à mettre AVANT la fonction)
direction = 0
direction_history = deque(maxlen=5)

def gen_frames():
    global direction, direction_history
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
        frame = cv2.resize(frame, (640, 480))
        h, w, _ = frame.shape
        results = model(frame, stream=False, verbose=False)
        r = results[0]
        
        if r.masks is None:
            _, buffer = cv2.imencode('.jpg', frame)
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            continue
        
        # === SEGMENTATION ===
        mask = r.masks.data[0].cpu().numpy()
        mask = cv2.resize(mask, (w, h))
        mask = (mask > 0.5).astype(np.uint8)
        
        mask_gray = (mask * 255).astype(np.uint8)
        mask_gray_bgr = cv2.cvtColor(mask_gray, cv2.COLOR_GRAY2BGR)
        frame = cv2.addWeighted(frame, 1.0, mask_gray_bgr, 0.4, 0)
        
        rows_with_road = np.where(mask.any(axis=1))[0]
        
        if len(rows_with_road) == 0:
            _, buffer = cv2.imencode('.jpg', frame)
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            continue
        
        y_top = rows_with_road[0]
        y_bottom = h
        y_box_bottom = rows_with_road[-1]
        cols_with_road = np.where(mask.any(axis=0))[0]

        if len(cols_with_road) > 0:
            x_left = cols_with_road[0]
            x_right = cols_with_road[-1]
            cv2.rectangle(frame, (x_left, y_top), (x_right, y_box_bottom), (255, 255, 0), 2)

            # === TRAJECTOIRE ===
            trajectory_points = []
            for y in range(y_top, y_bottom, 20):  # ← Du HAUT de la route au BAS de l'écran
                xs = np.where(mask[y] > 0)[0]
                if len(xs) > 0:
                    x_center = int(xs.mean())
                    trajectory_points.append((x_center, y))
                    cv2.circle(frame, (x_center, y), 4, (0, 255, 0), -1)

            if len(trajectory_points) > 1:
                cv2.polylines(
                    frame,
                    [np.array(trajectory_points)],
                    isClosed=False,
                    color=(0, 0, 255),
                    thickness=3
                )
        
        direction_raw = 0
        if len(trajectory_points) >= 2:
            # Point proche (bas)
            p_near = trajectory_points[-1]
            
            # Point lointain : premier tiers de la trajectoire
            # (75% d'anticipation environ)
            far_index = len(trajectory_points) // 5
            p_far = trajectory_points[far_index]
            
            dx = p_far[0] - p_near[0]
            dy = p_near[1] - p_far[1]
            angle_rad = np.arctan2(dx, dy)
            angle_deg = np.degrees(angle_rad)
            
            max_angle = sensibilite
            direction_raw = int(np.clip(angle_deg / max_angle, -1.0, 1.0) * 100)
        
        # === LISSAGE FORT ===
        direction_history.append(direction_raw)
        direction = int(np.mean(direction_history))
        
        
        cv2.putText(frame, f"Direction: {direction}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        
        cv2.line(frame, (w // 2, h), (w // 2, y_top), (255, 0, 0), 2)
        
        _, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()
        
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route("/")
def main():
    return render_template(
        "index.html")
    
@app.route("/data")
def data():
    return jsonify({
        "battery": battery,
        "gyro_x": gyro_x,
        "gyro_y": gyro_y,
        "gyro_z": gyro_z,
        "accel_x": accel_x,
        "accel_y": accel_y,
        "accel_z": accel_z,
        "temp": temp,
        "battery_percent": battery_percent,
        "vitesse": vitesse,
        "sensibilite": sensibilite,
        "distance_front": distance_front,
        "active": action_active
    })

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')
    
@app.route("/start", methods=["POST"])
def start_action():
    start_active()
    return jsonify({"status": "started", "action_active": action_active})

@app.route("/stop", methods=["POST"])
def stop_action():
    global action_active
    action_active = False
    return jsonify({"status": "stopped", "action_active": action_active})


@app.route("/set_vitesse", methods=["POST"])
def set_vitesse():
    global vitesse
    try:
        new_vitesse = int(request.args.get("vitesse", 15))
        vitesse = max(0, min(100, new_vitesse))
        return jsonify({"status": "vitesse set", "vitesse": vitesse})
    except ValueError:
        return jsonify({"status": "invalid value"}), 400

@app.route("/set_sensitivity", methods=["POST"])
def set_sensitivity():
    global sensibilite
    try:
        new_sensibility = float(request.args.get("sensitivity", 60.0))
        sensibilite = max(10.0, min(100.0, new_sensibility))
        return jsonify({"status": "sensitivity set", "sensitivity": sensibilite})
    except ValueError:
        return jsonify({"status": "invalid value"}), 400
    


if __name__ == "__main__":
    import logging
    logging.getLogger('werkzeug').setLevel(logging.ERROR)
    print("Flask Started")
    main_loop_thread = threading.Thread(target=main_loop, daemon=True)
    main_loop_thread.start()
    app.run(host="0.0.0.0", port=8080, threaded=True, debug=False)

