import serial
import time
import cv2
from ultralytics import YOLO




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
    return line.split(",")



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

def capture():
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
                    x = (x1+x2)/2
                    y = (y1+y2)/2
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

        cv2.imshow("YOLO Camera", frame)

        # ESC pour quitter
        if cv2.waitKey(1) & 0xFF == 27:
            break

capture()

while True :
    print(read_serial())
