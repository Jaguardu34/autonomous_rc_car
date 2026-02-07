from flask import Flask, render_template_string, request, Response
import serial
import time
import cv2
from threading import Thread
from ultralytics import YOLO

serial_buffer = ""  # variable globale pour stocker la dernière ligne


# =======================
# === CONFIG ROBOT ======
# =======================

vitesse_servo = 90

direction_base = 77
max_left = 50
max_right = 120

pos_vue = 90

pos_direction = direction_base

FORWARD = "forward"
BACKWARD = "backward"

# =======================
# === ARDUINO ===========
# =======================

def wait_for_arduino_ready(arduino, timeout=10):
    print("Attente de l'Arduino...")
    start_time = time.time()

    while time.time() - start_time < timeout:
        try:
            line = arduino.readline().decode().strip()
            if line == "##READY##":
                print("Arduino prêt!")
                return True
            if line:
                print(f"Arduino: {line}")
        except UnicodeDecodeError:
            pass

    print("Timeout Arduino")
    return False


arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
time.sleep(1)

if not wait_for_arduino_ready(arduino):
    arduino.close()
    exit(1)

def write_servo(a, b, c):
    arduino.write(f"{a}, {b}, {c}\r\n".encode())
    time.sleep(0.05)

# =======================
# === MOUVEMENTS ========
# =======================

def move(vitesse=0, sens=FORWARD):
    global vitesse_servo, pos_direction, pos_vue
    vitesse = max(0, min(100, vitesse))

    if sens == FORWARD:
        vitesse_servo = int(90 + vitesse * 90 / 100)
    elif sens == BACKWARD:
        vitesse_servo = int(90 - vitesse * 90 / 100)
    else:
        vitesse_servo = 90

    write_servo(vitesse_servo, pos_direction, pos_vue)

def stop():
    global vitesse_servo, pos_direction, pos_vue
    vitesse_servo = 90
    write_servo(vitesse_servo, pos_direction, pos_vue)

def left(degres):
    global vitesse_servo, pos_direction, pos_vue
    degres = max(0, min(100, degres))
    pos_direction = int(direction_base - (degres * (direction_base - max_left) / 100))
    write_servo(vitesse_servo, pos_direction, pos_vue)

def right(degres):
    global vitesse_servo, pos_direction, pos_vue
    degres = max(0, min(100, degres))
    pos_direction = int(direction_base + (degres * (max_right - direction_base) / 100))
    write_servo(vitesse_servo, pos_direction, pos_vue)

def center():
    global vitesse_servo, pos_direction, pos_vue
    pos_direction = direction_base
    write_servo(vitesse_servo, pos_direction, pos_vue)
    
def look_right(a):
    global vitesse_servo, pos_direction, pos_vue
    if pos_vue - a >= 0:
        pos_vue -= a
    else:
        pos_vue = 0
    write_servo(vitesse_servo, pos_direction, pos_vue)
        
def look_left(a):
    global vitesse_servo, pos_direction, pos_vue
    if pos_vue + a <= 180:
        pos_vue += a
    else:
        pos_vue = 180
    write_servo(vitesse_servo, pos_direction, pos_vue)
        
def look_center():
    global vitesse_servo, pos_direction, pos_vue
    pos_vue = 90
    write_servo(vitesse_servo, pos_direction, pos_vue)
    
def read_serial():
    line = arduino.readline().decode().strip()
    return line

def serial_reader():
    global serial_buffer
    while True:
        try:
            line = arduino.readline().decode().strip()
            if line:
                serial_buffer = line + "V"
        except UnicodeDecodeError:
            pass

# lance le thread
Thread(target=serial_reader, daemon=True).start()

write_servo(90, direction_base, pos_vue)

model = YOLO("yolo-Weights/yolo11n.pt")
model.fuse()

# Object Classes
classNames = ["person","bicycle","car","motorbike","aeroplane","bus","train","truck","boat",
              "traffic light","fire hydrant","stop sign","parking meter","bench","bird","cat",
              "dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack","umbrella",
              "handbag","tie","suitcase","frisbee","skis","snowboard","sports ball","kite","baseball bat",
              "baseball glove","skateboard","surfboard","tennis racket","bottle","wine glass","cup",
              "fork","knife","spoon","bowl","banana","apple","sandwich","orange","broccoli",
              "carrot","hot dog","pizza","donut","cake","chair","sofa","pottedplant","bed",
              "diningtable","toilet","tvmonitor","laptop","mouse","remote","keyboard","cell phone",
              "microwave","oven","toaster","sink","refrigerator","book","clock","vase","scissors",
              "teddy bear","hair drier","toothbrush"]

classNamesShow = ["person"]



# --------- Video Capture ---------
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


if not cap.isOpened():
    raise RuntimeError("Impossible d’ouvrir la caméra")

def gen_frames():
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        results = model(frame, device=0, half=True)

        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])

                if classNames[cls] in classNamesShow and conf >= 0.4:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    x = (x1-x2)/2
                    y = (y1-y2)/2
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 2)
                    cv2.putText(
                        frame,
                        f"{classNames[cls]} {conf:.2f} x:{x} y:{y}",
                        (x1, y1 - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 0, 0),
                        2
                    )

        # Encode frame as JPEG
        _, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()

        # MJPEG stream
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

# =======================
# === FLASK =============
# =======================

app = Flask(__name__)

HTML_PAGE = """
<!DOCTYPE html>
<html>
<head>
<title>Robot Control</title>
<style>
body { background:#111; color:#eee; font-family:Arial; text-align:center; }
button { width:70px; height:70px; font-size:24px; margin:5px; }
img { border:3px solid #555; margin-top:10px; }
</style>
</head>
<body>

<h2>Contrôle Robot</h2>

<div>
<button onclick="send('forward')">↑</button><br>
<button onclick="send('left')">←</button>
<button onclick="send('stop')">■</button>
<button onclick="send('right')">→</button><br>
<button onclick="send('backward')">↓</button>
</div>

<h3>Caméra</h3>
<img src="/video_feed" width="640" height="480">

<h3>Arduino Serial Output</h3>
<div id="serial_output" style="background:#222; color:#0f0; padding:10px; font-family:monospace; height:30px;"></div>

<script>
function send(cmd){
    fetch('/control?cmd=' + cmd);
}

document.addEventListener("keydown", function(e){
    if(e.repeat) return;
    switch(e.key){
        case "ArrowUp":
        case "z": send("forward"); break;
        case "ArrowDown":
        case "s": send("backward"); break;
        case "ArrowLeft":
        case "q": send("left"); break;
        case "ArrowRight":
        case "d": send("right"); break;
        case " ": send("stop"); break;
        case "a": send("look_left"); break;
        case "e": send("look_right"); break;
        case "r": send("look_center"); break;
    }
});

document.addEventListener("keyup", function(e){
    if(["ArrowUp","ArrowDown","z","s"].includes(e.key)){
        send("stop");
    }
    if(["ArrowLeft","ArrowRight","q","d"].includes(e.key)){
        send("center");
    }
});

function updateSerial() {
    fetch('/serial_line')
        .then(response => response.text())
        .then(data => {
            document.getElementById('serial_output').innerText = data;
        })
        .catch(err => console.error(err));
}

// rafraîchit toutes les 200 ms
setInterval(updateSerial, 50); // toutes les 50 ms

</script>

</body>
</html>
"""

@app.route("/")
def index():
    return render_template_string(HTML_PAGE)

@app.route("/control")
def control():
    cmd = request.args.get('cmd')

    if cmd == "forward":
        move(15, FORWARD)
    elif cmd == "backward":
        move(15, BACKWARD)
    elif cmd == "left":
        left(100)
    elif cmd == "right":
        right(100)
    elif cmd == "stop":
        stop()
    elif cmd == "center":
        center()
    elif cmd == "look_left":
        look_left(10)
    elif cmd == "look_right":
        look_right(10)
    elif cmd == "look_center":
        look_center()

    return "OK"

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route("/serial_line")
def serial_line():
    return serial_buffer

# =======================
# === MAIN ==============
# =======================

if __name__ == "__main__":
    write_servo(90, direction_base, pos_vue)
    app.run(host="0.0.0.0", port=5000, threaded=True)
