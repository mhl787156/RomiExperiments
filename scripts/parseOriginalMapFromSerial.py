import serial
import os
import datetime

PORT = "COM7"
filename = "data/expr1-{}.map".format(datetime.datetime.now().strftime("%Y-%m-%d_%H-%M"))

mymap = ""
foundmap = False
with serial.Serial('\\.\{}'.format(PORT)) as ser:
    for i in range(1000):
        raw = ser.readline()
        decode = raw.decode('utf-8').strip()
        if decode == 'Map':
            if foundmap:
                break
            else:
                os.system('cls')
                mymap = ""
                foundmap = True
        else:
            print(decode)
            mymap =  mymap + decode + '\n'

with open(filename, 'w') as f:
    f.write(mymap)