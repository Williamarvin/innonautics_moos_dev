#************************************************************#
#*    NAME: Blake Cole                                      *#
#*    ORGN: MIT                                             *#
#*    FILE: serial_ais.py                                   *#
#*    DATE: 29 MARCH 2021                                   *#
#************************************************************#

# SERIAL_AIS appends serial data to a text file

#adapted from:
# Eli The Computer Guy
# "Arduino – Read Serial Communication with Raspberry Pi"
# <https://www.elithecomputerguy.com/>

import serial
import time


if __name__ == '__main__':
    # open logfile, write header row
    fname = r"/home/pi/moos-ivp-pavlab/data/ais.log"
    header = "timestamp,payload \n"
    logfile = open(fname, "w")
    logfile.write(header)
    logfile.close()
    
    # initiate serial reading, flush buffer
    s = serial.Serial('/dev/serial0', baudrate=38400, timeout=1)
    s.flush()

    logging = True

    while logging:
        try:
            if (s.in_waiting > 0):
                current_time = time.strftime("%Y-%m-%d %H:%M:%S")

                # parse payload
                payload = s.readline().decode('utf-8').strip()
                #msg = dict(item.split("=") for item in payload.split(","))
                #line = current_time + "," + ','.join(map(str, msg.values()))
                line = current_time + "," + payload
                print(line)

                # save csv line to file
                logfile = open(fname, "a")
                logfile.write(line + "\n")
                logfile.close()

        except KeyboardInterrupt:
            print("\n")
            print("[data logging stopped by user]\n");
            logging = False
            logfile.close()
