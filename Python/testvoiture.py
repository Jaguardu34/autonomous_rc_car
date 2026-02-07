import control
import time


control.init_serial()


while True:
    print("Test de mouvement avant")
    control.move(60, "f")   # pas 10, pas 20
    time.sleep(1)

    print("Stop")
    control.move(60, "b")
    time.sleep(0.5)
    control.stop()
    time.sleep(0.5)
    print("Test de mouvement arrière")
    control.move(15, "b")
    time.sleep(2)
    control.stop()
    
    print("Test termine - redemarrage dans 5 secondes")
    time.sleep(5)