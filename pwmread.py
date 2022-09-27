import RPi.GPIO as GPIO
import time

pin = 12

GPIO.setmode(GPIO.BCM)
GPIO.setup(pin, GPIO.IN, pull_up_down = GPIO.PUD_UP)

try:
    while True:
        time.sleep(0.017)
        while GPIO.input(pin) == 0:
            pass
        t = time.time()
        while GPIO.input(pin) == 1:
            pass
        diff = time.time() - t
        print (round(10 + 1000000 * (diff)))
        while (round(10 + 1000000 * (diff)) > 2010):
            print("Inside")

finally:
    GPIO.cleanup()