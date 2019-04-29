import serial
import os
import datetime
import matplotlib.pyplot as plt
import numpy
import time
import string

filePrepend = "Hand_Test"

foldername = "expr1-{}-{}".format(filePrepend, datetime.datetime.now().strftime("%Y-%m-%d_%H-%M"))

PORT = "COM12"
datafilename = "data/{}/dat.data".format(foldername)
mapfilename = "data/{}/map.map".format(foldername)

timelimit = 60 * 3

print("Script up finding ROMI on Bluetooth {}".format(PORT))
mapString = ""
dataString = ""
with serial.Serial(PORT, timeout=3) as ser:
    try:
        input('Press Enter to continue')
        ser.write(b'A')
        print('Romi Started')
        starttime = time.time()

        stoploop = False
        while True:
            if time.time() - starttime > timelimit:
                print('TIME LIMIT REACHED, Stopping ROMI')
                ser.write(b'S')
                stoploop = True

            current_time = datetime.datetime.now().strftime("%H-%M-%S")
            raw = ser.readline()
            try:
                decode = raw.decode('utf-8').strip()
            except:
                decode = "Failed to read line"
            
            print(current_time, decode)

            # Decode the map
            if 'Map' in decode and 'Mapping' not in decode:
                if 'Raw' in decode:
                    printstr = 'RawMap-{}\n'.format(current_time) 
                else:
                    printstr = 'HumanMap-{}\n'.format(current_time) 
                mymap = printstr
                for i in range(25):
                    raw = ser.readline()
                    try:
                        decode = raw.decode('utf-8').strip()
                        # if not all(c in string.printable for c in decode):
                        #     raise RuntimeError("Odd Character")
                    except Exception:
                        decode = "Failed to read line in map"
                    print(decode)
                    mymap =  mymap + decode + '\n'
                mapString = mapString + mymap + '\n' 
                if stoploop:
                    break             
            else:
                dataString = dataString + decode + '\n'                
    except KeyboardInterrupt:
        print('Stopping and Saving')
    except Exception as e:
        print('Error occured')
        print(e)
            
x = input('Save Data (y/n)?: ')
if(x in 'Yy'):
    os.mkdir("data/{}".format(foldername))
    with open(mapfilename, 'w+') as f:
        f.write(mapString)
    print('Saved Map in {}'.format(mapfilename))
    with open(datafilename, 'w+') as f:
        f.write(dataString)
    print('Saved Data in {}'.format(datafilename))
    
