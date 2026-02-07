import serial
import time
import threading

# Variables
vitesse_servo = 90
direction_base = 77
max_left = 50
max_right = 120
pos_direction = direction_base
last_receive_time = time.time()
arduino = None
is_connected = False
running = False
monitor_thread = None
lock = threading.Lock()

def init_serial(port='/dev/ttyACM0', baud=115200, timeout=0.2):
    global arduino, is_connected, last_receive_time
    with lock:
        if arduino and arduino.is_open:
            arduino.close()
            time.sleep(0.5)
        
        arduino = serial.Serial(port, baud, timeout=timeout)
        arduino.setDTR(False)
        time.sleep(0.1)
        arduino.setDTR(True)
        time.sleep(4)
        arduino.reset_input_buffer()
        
        is_connected = True
        last_receive_time = time.time()

def start_monitoring(timeout=5.0, check_interval=1.0):
    global running, monitor_thread
    running = True
    monitor_thread = threading.Thread(
        target=_monitor_loop, 
        args=(timeout, check_interval), 
        daemon=True
    )
    monitor_thread.start()

def stop_monitoring():
    global running, monitor_thread
    running = False
    if monitor_thread:
        monitor_thread.join(timeout=2)

def _monitor_loop(timeout, check_interval):
    global last_receive_time, is_connected
    while running:
        time.sleep(check_interval)
        if time.time() - last_receive_time > timeout:
            if is_connected:
                print(f"⚠️ Timeout {timeout}s - Test connexion...")
                _self_test()

def _self_test():
    global arduino, is_connected, last_receive_time
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
                print("✗ Pas de réponse")
                is_connected = False
                reconnect_serial()
        except Exception as e:
            print(f"✗ Erreur test: {e}")
            is_connected = False
            reconnect_serial()

def write_servo(a, b):
    global arduino, is_connected
    with lock:
        if not is_connected or not arduino:
            reconnect_serial()
        try:
            arduino.write(f"{a},{b}\r\n".encode())
        except Exception:
            is_connected = False

def move(vitesse=0, sens="f"):
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
    global vitesse_servo, pos_direction
    vitesse_servo = 90
    write_servo(vitesse_servo, pos_direction)

def left(degres):
    global pos_direction
    degres = max(0, min(100, degres))
    pos_direction = int(direction_base - (degres * (direction_base - max_left) / 100))
    write_servo(vitesse_servo, pos_direction)

def right(degres):
    global pos_direction
    degres = max(0, min(100, degres))
    pos_direction = int(direction_base + (degres * (max_right - direction_base) / 100))
    write_servo(vitesse_servo, pos_direction)

def center():
    global pos_direction
    pos_direction = direction_base
    write_servo(vitesse_servo, pos_direction)

def read_serial():
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

def reconnect_serial():
    global arduino, is_connected
    try:
        if arduino:
            arduino.close()
    except Exception:
        pass
    
    print("🔄 Reconnexion...")
    while running:
        try:
            init_serial()
            print("✓ Reconnecté")
            return
        except Exception:
            time.sleep(1)

def safe_read():
    try:
        return read_serial()
    except Exception as e:
        print(f"Erreur série: {e}")
        is_connected = False
        reconnect_serial()
        return None