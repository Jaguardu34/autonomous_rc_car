import serial
import time
import threading

# ---------------- Variables globales ----------------
vitesse_servo = 90
pos_direction = 77
direction_base = 77
max_left = 50
max_right = 120

reconnecting= False

arduino = None
is_connected = False
last_receive_time = 0
running = False
monitor_thread = None
lock = threading.Lock()

# ---------------- Initialisation série ----------------
def init_serial(port='/dev/ttyACM0', baud=115200, timeout=0.2):
    """Initialise la connexion série avec Arduino"""
    global arduino, last_receive_time, is_connected
    if arduino and arduino.is_open:
        is_connected = False
        arduino.close()
        time.sleep(0.2)
    try: 
        arduino = serial.Serial(port, baud, timeout=timeout)
    except serial.SerialException as e:
        print(f"⚠️ SerialException: {e}")
        is_connected = False
        return False
    arduino.setDTR(False)
    time.sleep(0.05)
    arduino.setDTR(True)
    if not wait_for_boot(timeout=5.0):
        print("⚠️ L'Arduino n'a pas envoyé BOOT après init")
        is_connected = False
        return False
    arduino.reset_input_buffer()
    last_receive_time = time.time()
    print("✓ Serial initialisé")
    is_connected = True
    return True



def reconnect_serial():
    """Reconnecte l'Arduino avec un backoff progressif"""
    global arduino, is_connected, reconnecting
    
    if reconnecting:
        return False
    
    reconnecting = True
    is_connected = False
    
    if arduino:
        try:
            arduino.close()
        except:
            pass
    print("🔄 Tentative de reconnexion...")
    try:
        if init_serial():
            print("✓ Reconnecté")
        reconnecting = False
        return True
    except Exception:
        print("Tentative échouée")
        time.sleep(0.5)
    print("✗ Impossible de reconnecter")
    reconnecting = False
    is_connected = False
    return False

def wait_for_boot(timeout=10.0):
    """Attend que l'Arduino envoie 'BOOT,1.0' sur le port série"""
    global arduino, last_receive_time, is_connected
    start_time = time.time()
    
    while True:
        if arduino is None or not arduino.is_open:
            return False

        try:
            line = arduino.readline().decode(errors='ignore').strip()
        except serial.SerialException:
            return False

        if line:
            # Detection du BOOT
            if line.startswith("BOOT"):
                print(f"🔁 Arduino boot détecté: {line}")
                last_receive_time = time.time()
                is_connected = True
                return True
        # Timeout
        if time.time() - start_time > timeout:
            print("⚠️ Timeout waiting for BOOT")
            return False


# ---------------- Commandes servos ----------------
def write_servo(a, b):
    global arduino, is_connected
    if arduino and is_connected:
        arduino.write(f"{a},{b}\r\n".encode())

def move(vitesse=0, sens="f"):
    """Déplace le moteur vers l'avant ou l'arrière"""
    global vitesse_servo, pos_direction
    vitesse = max(0, min(100, vitesse))
    if sens == "f":
        vitesse_servo = int(90 + vitesse * 90 / 100)
    elif sens == "b":
        vitesse_servo = int(90 - vitesse * 90 / 100)
    else:
        vitesse_servo = 90
    write_servo(vitesse_servo, pos_direction)

def stop():
    """Arrête le moteur"""
    global vitesse_servo, pos_direction
    vitesse_servo = 90
    write_servo(vitesse_servo, pos_direction)

def left(degres):
    """Tourne la direction à gauche"""
    global pos_direction
    degres = max(0, min(100, degres))
    pos_direction = int(direction_base - (degres * (direction_base - max_left) / 100))
    write_servo(vitesse_servo, pos_direction)

def right(degres):
    """Tourne la direction à droite"""
    global pos_direction
    degres = max(0, min(100, degres))
    pos_direction = int(direction_base + (degres * (max_right - direction_base) / 100))
    write_servo(vitesse_servo, pos_direction)

def center():
    """Recentre la direction"""
    global pos_direction
    pos_direction = direction_base
    write_servo(vitesse_servo, pos_direction)

# ---------------- Lecture série ----------------
def read_serial():
    global arduino, last_receive_time, is_connected
    now = time.time()
    if is_connected and arduino:
        try:
            line = arduino.readline().decode(errors='ignore').strip()
        except serial.SerialException as e:
            print(f"⚠️ SerialException: {e}")
            is_connected = False
            try:
                arduino.close()
            except:
                pass
            return None
        if not line:
            is_connected = False
            if now - last_receive_time > 0.5:
                reconnect_serial()
            return None
        else:
            is_connected = True
            last_receive_time = time.time()
            parts = line.split(',')
            return [float(p) for p in parts]
    else: 
        is_connected = False
        if now - last_receive_time > 0.5:
            reconnect_serial()
        return None


