import GPIO
import time

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

# Start the periodic trigger loop
next_time = time.monotonic()
while True:
    GPIO.write(FSIN_2LANE, GPIO.HIGH)
    time.sleep(ACTIVE_INTERVAL)
    GPIO.write(FSIN_2LANE, GPIO.LOW)
    next_time += CAPTURE_INTERVAL
    wait_time = next_time - time.monotonic()
    if wait_time < 0:
        node.warn(f'FSYNC is late by {{-wait_time * 1e-6}}us!')
    else:
        time.sleep(wait_time)
