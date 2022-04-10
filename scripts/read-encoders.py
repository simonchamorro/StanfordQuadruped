
import time
import numpy as np
import pigpio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from adafruit_extended_bus import ExtendedI2C as I2C


# Key = leg_idx, motor_idx
# Value = motor bin number for mux
MOTOR2BIN = {(0, 0):  [0, 0, 0],
             (0, 1):  [1, 0, 0],
             (0, 2):  [0, 1, 0],
             (1, 0):  [1, 1, 0],
             (1, 1):  [0, 0, 1],
             (1, 2):  [1, 0, 1],
             (2, 0):  [0, 0, 0],
             (2, 1):  [1, 0, 0],
             (2, 2):  [0, 1, 0],
             (3, 0):  [1, 1, 0],
             (3, 1):  [0, 0, 1],
             (3, 2):  [1, 0, 1],}

# GPIO pins, not the same as pin numbers
# see: http://abyz.me.uk/rpi/pigpio/index.html#Type_3
MUX_PINS_FRONT = [16, 26, 19]
# MUX_PINS_BACK = [6, 5, 0]

# Neutral positions
NEUTRAL_POS = [0, 45, -45]


def set_motor(pi, motor=0, mux_pins=[]):
    bin_num = MOTOR2BIN[motor]
    for idx, pin in enumerate(mux_pins):
        pi.write(pin, bin_num[idx])

def get_motor_name(i, j):
    motor_type = {0: "hip", 1: "thigh", 2: "calf"} 
    leg_pos = {0: "front-right", 1: "front-left", 2: "back-right", 3: "back-left"}
    final_name = motor_type[i] + " " + leg_pos[j]
    return final_name

def main():
    # Create the I2C bus
    i2c = I2C(4)

    # Create Pi object
    pi = pigpio.pi()

    # Create the ADC object using the I2C bus
    ads = ADS.ADS1015(i2c)

    # Create single-ended input on channel 0
    chan0 = AnalogIn(ads, ADS.P0)
    chan1 = AnalogIn(ads, ADS.P1)

    # Load encoders calibration
    encoder_calib = np.load("encoder_config.npz")["encoder_calib"]

    while True:
        for leg_idx in range(4):
            for motor_idx in range(3):
                # Set the mux to desired motor
                set_motor(pi, (leg_idx, motor_idx), MUX_PINS_FRONT)
                # set_motor(pi, (leg_idx, motor_idx), MUX_PINS_BACK)

                # Choose right channel and read value
                if motor_idx < 2:
                    chan = chan0
                else:
                    chan = chan1
                enc_reading = chan.voltage

                # Convert to angle value
                neutral_voltage = encoder_calib[leg_idx, motor_idx, 0]
                neutral_angle = NEUTRAL_POS[motor_idx]
                volt_deg_ratio = encoder_calib[leg_idx, motor_idx, 1]
                enc_angle = (enc_reading - neutral_voltage) / volt_deg_ratio + neutral_angle

                print("\n===========================\n")
                motor_name = get_motor_name(motor_idx, leg_idx)
                print(motor_name)
                print("Encoder angle: {}".format(enc_angle))
                
        time.sleep(1.0)



if __name__ == "__main__":
    main()    