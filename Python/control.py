import serial
import time
import threading

# ---------------- Variables globales ----------------
vitesse_servo = 90
pos_direction = 77
direction_base = 77
max_left = 50
max_right = 120

arduino = None
is_connected = False
last_receive_time = 0
running = False
monitor_thread = None
lock = threading.Lock()

# ---------------- Initialisation série ----------------
def init_serial(port='/dev/ttyACM0', baud=115200, timeout=0.2):
    """Initialise la connexion série avec Arduino"""
    global arduino, is_connected, last_receive_time
    with lock:
        if arduino and arduino.is_open:
            arduino.close()
            time.sleep(0.2)
        arduino = serial.Serial(port, baud, timeout=timeout)
        arduino.setDTR(False)
        time.sleep(0.1)
        arduino.setDTR(True)
        time.sleep(2)  # reset Arduino
        arduino.reset_input_buffer()
        is_connected = True
        last_receive_time = time.time()
        print("✓ Serial initialisé")

# ---------------- Monitoring et reconnexion ----------------
def start_monitoring(timeout=5.0, check_interval=1.0):
    """Démarre le thread de surveillance de la connexion"""
    global running, monitor_thread
    running = True
    monitor_thread = threading.Thread(
        target=_monitor_loop, args=(timeout, check_interval), daemon=True
    )
    monitor_thread.start()

def stop_monitoring():
    """Arrête le thread de surveillance"""
    global running, monitor_thread
    running = False
    if monitor_thread:
        monitor_thread.join(timeout=2)
        monitor_thread = None

def _monitor_loop(timeout, check_interval):
    """Boucle interne de surveillance"""
    global last_receive_time, is_connected
    while running:
        time.sleep(check_interval)
        if time.time() - last_receive_time > timeout:
            print(f"⚠️ Timeout {timeout}s - Vérification connexion...")
            _self_test()

def _self_test():
    """Test simple de communication avec l'Arduino"""
    global last_receive_time, is_connected
    with lock:
        if not arduino or not arduino.is_open:
            is_connected = False
            reconnect_serial()
            return
        try:
            arduino.write(b"90,77\r\n")
            old_timeout = arduino.timeout
            arduino.timeout = 0.5
            response = arduino.readline()
            arduino.timeout = old_timeout
            if response:
                last_receive_time = time.time()
                is_connected = True
                print("✓ Arduino OK")
            else:
                is_connected = False
                reconnect_serial()
        except Exception as e:
            print(f"✗ Erreur test: {e}")
            is_connected = False
            reconnect_serial()

def reconnect_serial(max_attempts=10):
    """Reconnecte l'Arduino avec un backoff progressif"""
    global arduino, is_connected
    if arduino:
        try:
            arduino.close()
        except:
            pass
    print("🔄 Tentative de reconnexion...")
    for attempt in range(max_attempts):
        if not running:
            break
        try:
            init_serial()
            print("✓ Reconnecté")
            return True
        except Exception:
            print(f"⏳ Tentative {attempt+1}/{max_attempts} échouée")
            time.sleep(1 + attempt*0.5)
    print("✗ Impossible de reconnecter")
    is_connected = False
    return False

# ---------------- Commandes servos ----------------
def write_servo(a, b):
    """Envoie les positions à l'Arduino"""
    global arduino, is_connected
    with lock:
        if not is_connected or not arduino:
            print("⚠️ Arduino non connecté, tentative de reconnexion...")
            reconnect_serial()
        try:
            arduino.write(f"{a},{b}\r\n".encode())
        except Exception as e:
            print(f"Erreur write_servo: {e}")
            is_connected = False

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
    """Lit les données de l'Arduino"""
    global arduino, last_receive_time, is_connected
    with lock:
        if arduino is None:
            raise serial.SerialException("Port non initialisé")
        try:
            line = arduino.readline().decode(errors='ignore').strip()
            if not line:
                return None
            parts = line.split(',')
            if len(parts) != 9:
                return None
            last_receive_time = time.time()
            is_connected = True
            return [float(p) for p in parts]
        except serial.SerialException:
            is_connected = False
            raise

def safe_read():
    """Lecture sécurisée, reconnecte si nécessaire"""
    try:
        return read_serial()
    except Exception as e:
        print(f"Erreur série: {e}")
        is_connected = False
        reconnect_serial()
        return None
