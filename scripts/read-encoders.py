import time
import pigpio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from adafruit_extended_bus import ExtendedI2C as I2C

MOTOR2BIN = {0:  [0, 0, 0, 0],
             1:  [1, 0, 0, 0],
             2:  [0, 1, 0, 0],
             3:  [1, 1, 0, 0],
             4:  [0, 0, 1, 0],
             5:  [1, 0, 1, 0],
             6:  [0, 1, 1, 0],
             7:  [1, 1, 1, 0],
             8:  [0, 0, 0, 1],
             9:  [1, 0, 0, 1],
             10: [0, 1, 0, 1],
             11: [1, 1, 0, 1],}

# GPIO pins, not the same as pin numbers
# see: http://abyz.me.uk/rpi/pigpio/index.html#Type_3
MUX_PINS = [16, 26, 6, 5]

def set_motor(pi, motor=0):
    bin_num = MOTOR2BIN[motor]
    for idx, pin in enumerate(MUX_PINS):
        pi.write(pin, bin_num[idx])

# Create the I2C bus
i2c = I2C(4)

# Create Pi object
pi = pigpio.pi()

# Create the ADC object using the I2C bus
ads = ADS.ADS1015(i2c)

# Create single-ended input on channel 0
chan = AnalogIn(ads, ADS.P0)

while True:

    print("\n===========================\n")
    print('Front Right')
    for i in range(3):
        set_motor(pi, motor=i)
        print("{:>5}\t{:>5.3f}".format(i, chan.voltage))
    
    print('Front Left')
    for i in range(3):
        set_motor(pi, motor=i+3)
        print("{:>5}\t{:>5.3f}".format(i, chan.voltage))
    
    print('Rear Right')
    for i in range(3):
        set_motor(pi, motor=i+6)
        print("{:>5}\t{:>5.3f}".format(i, chan.voltage))
    
    print('Rear Left')
    for i in range(3):
        set_motor(pi, motor=i+9)
        print("{:>5}\t{:>5.3f}".format(i, chan.voltage))
    
    time.sleep(1.0)