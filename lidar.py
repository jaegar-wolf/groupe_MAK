import signal, sys
from rplidar import RPLidar
from time import sleep

LIDAR_PORT = '/dev/ttyUSB0'

result = []

while(True):
    print('starting...')
    sleep(2)
    try :
        lidar = RPLidar(LIDAR_PORT)
        for i, scan in enumerate(lidar.iter_scans()):
            for qual, angle, dist in scan:
                print(angle, dist)
                result.append((int(angle), int(dist)))
            break

        print(result)
        print(len(result))   
        # for (i, val) in enumerate(lidar.iter_measurments()):
        #     _, qual, angle, dist = val
        #     print(qual, angle, dist)
        #     break
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        break
    except KeyboardInterrupt:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        break
    except SystemExit:
        break
    except:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        try:
            print(str(exc_type) + '\t' + str(exc_value))
        except KeyboardInterrupt:
            lidar = RPLidar(LIDAR_PORT)
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()