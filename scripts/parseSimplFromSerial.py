import serial
import os
import datetime
import numpy


PORT = "COM12"
print("Script up finding ROMI on Bluetooth {}".format(PORT))

with serial.Serial(PORT, timeout=1) as ser:
    try:
        while True:
            current_time = datetime.datetime.now().strftime("%H-%M-%S")
            raw = ser.readline()
            try:
                decode = raw.decode('utf-8').strip()
                if decode != "":
                    print(current_time, decode)
            except UnicodeDecodeError as e:
                print("ERROR", e)
                print(raw)

    except KeyboardInterrupt:
        print('Stopping and Saving')
            