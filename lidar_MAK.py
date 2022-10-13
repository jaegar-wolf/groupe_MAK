from numpy import result_type
from rplidar import RPLidar
import math as math
import numpy as np
from time import sleep
import signal, sys
import csv

def acquisition():
    result = []
    while(True):
        try:
            sleep(1) 
            lidar = RPLidar('/dev/ttyUSB0')
            # # loop for the scan
            print('Wait few seconds for the data generation ')
            for i, (new_scan, quality, angle, distance) in enumerate(lidar.iter_measurments(200)):
                # Data file creation
                if i == 200:
                    break
                result.append((int(angle),int(distance)))
                #print(angle,distance)
           # print(result)
            break
        
        except Exception:
    
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
            exc_type, exc_value, exc_traceback = sys.exc_info()       
            print(str(exc_type) + '\t' + str(exc_value))
    return result

#acquisition() 
