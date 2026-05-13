import GPIO
import time
import threading

# Script static arguments passed from C++/Python
BOARD_REVISION = {}
CAPTURE_INTERVAL = {}
ACTIVE_INTERVAL = 0.001

# Configure GPIO pins based on revision
FSIN_2LANE = 41  # COM_AUX_IO2
FSIN_4LANE = 42
if BOARD_REVISION < 6:
    FSIN_4LANE = 40
GPIO.setup(FSIN_4LANE, GPIO.IN)
GPIO.setup(FSIN_2LANE, GPIO.OUT)
GPIO.write(FSIN_2LANE, GPIO.LOW)

def periodic_trigger():
    global next_time
    GPIO.write(FSIN_2LANE, GPIO.HIGH)
    threading.Timer(ACTIVE_INTERVAL, lambda: GPIO.write(FSIN_2LANE, GPIO.LOW)).start()
    next_time += CAPTURE_INTERVAL

    wait_time = next_time - time.monotonic()
    if wait_time < 0:
        node.warn(f'FSYNC is late by {{-wait_time * 1e-6}}us!')

    threading.Timer(next_time - time.monotonic(), periodic_trigger).start()

# Start the periodic trigger loop
next_time = time.monotonic()
periodic_trigger()
