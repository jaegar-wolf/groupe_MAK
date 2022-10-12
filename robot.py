# à lancer sur le robot

from itertools import count
from multiprocessing.resource_sharer import stop
import serial
from time import sleep
import os

def instruction(path1, path2):
    while(True):
        robotPort = os.popen('ls /dev/ttyACM*').read().strip()
        print("Found serial : " + robotPort)

        def openSerial():
            return serial.Serial(robotPort)

        def sendInstruction(char, ser):
            ser.write(char.encode())

        ser = openSerial()

        def moveAhead(lettre, stop):
            count = 0
            for i in range(8):
                sleep(0.1)
                sendInstruction(lettre, ser)
                count += 1
            if count == 8:
                sendInstruction(stop, ser)
                sendInstruction(stop, ser)

        def moveAside(lettre, stop):
            count = 0
            for i in range(20):
                sleep(0.1)
                sendInstruction(lettre, ser)
                count += 1
            if count == 20:
                sendInstruction(stop, ser)
                sendInstruction(stop, ser)
           
        if path1[0]==path2[0] and path1[1]<path2[1]:
            moveAhead('z', 's')
            print('avance')
                
        elif  path1[0]>path2[0] and path1[1]==path2[1]:  
            moveAside('q', 'd')
            print('gauche')
            
        elif  path1[0]<path2[0] and path1[1]==path2[1]: 
            moveAside('d', 'q')
            print('droite')
    
        elif  path1[0]>path2[0] and path1[1]>path2[1]: 
            moveAhead('s', 'z')
            print('recule')
                
                
        ser.close()
        break

