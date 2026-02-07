import serial
import threading

# Remplace par ton port série et le baud rate de l'Arduino
SERIAL_PORT = '/dev/ttyTHS1'
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
except Exception as e:
    print(f"Impossible d'ouvrir le port {SERIAL_PORT} :", e)
    exit(1)

def read_serial():
    """Thread qui lit le port série et affiche les messages Arduino."""
    while True:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').rstrip()
            if line:
                print(f"\nArduino: {line}")
        except Exception:
            pass

# Démarrer le thread de lecture série
threading.Thread(target=read_serial, daemon=True).start()

print(f"Port {SERIAL_PORT} ouvert. Tape tes commandes. Ctrl+C pour quitter.")

try:
    while True:
        cmd = input("> ")  # lire les commandes clavier
        ser.write((cmd + "\r\n").encode('utf-8'))  # envoi avec \r\n
except KeyboardInterrupt:
    print("\nFermeture du port série...")
finally:
    ser.close()
