import serial
import os
import datetime
import matplotlib.pyplot as plt
import numpy

filePrepend = "Belief_Human_Readable"

PORT = "COM12"
datafilename = "data/expr1-{}-{}.data".format(filePrepend, datetime.datetime.now().strftime("%Y-%m-%d_%H-%M"))
mapfilename = "data/expr1-{}-{}.map".format(filePrepend, datetime.datetime.now().strftime("%Y-%m-%d_%H-%M"))

# hl, = plt.plot([], [])

# def update_plot(h1, new_data):
#     hl.set_xdata(numpy.append(hl.get_xdata(), new_data))
#     hl.set_ydata(numpy.append(hl.get_ydata(), new_data))
#     plt.draw()

# update_plot(hl, (900, 900))

print("Script up finding ROMI on Bluetooth {}".format(PORT))
mapString = ""
dataString = ""
with serial.Serial(PORT, timeout=3) as ser:
    try:
        while True:
            current_time = datetime.datetime.now().strftime("%H-%M-%S")
            raw = ser.readline()
            decode = raw.decode('utf-8').strip()
            print(current_time, decode if decode != "" else "TIMEOUT")

            # Decode the map
            if 'Map' in decode and 'Mapping' not in decode:
                printstr = 'Map-{}\n'.format(current_time) 
                mymap = printstr
                for i in range(25):
                    raw = ser.readline()
                    decode = raw.decode('utf-8').strip()
                    print(decode)
                    mymap =  mymap + decode + '\n'
                mapString = mapString + mymap + '\n'              
            else:
                dataString = dataString + decode + '\n'               
                

    except KeyboardInterrupt:
        print('Stopping and Saving')
            

with open(mapfilename, 'w+') as f:
    f.write(mapString)
with open(datafilename, 'w+') as f:
    f.write(dataString)
