"""
Get the orientation and GPS values from the rover hardware
"""
import time
import math
from pymavlink import mavutil

def read_gps(master):
    try:
        print("Reading GPS Values")

        while True:
            gps_raw_int_msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=0.1)
            gps_raw_int = None

            if gps_raw_int_msg:
                data = gps_raw_int_msg.to_dict()
                lat = data['lat'] / 1e7
                lon = data['lon'] / 1e7
                alt = data['alt'] / 1e3
                gps_raw_int = [lat, lon, alt]
                #print("read gps!!!")
            yield(gps_raw_int)
    except Exception as e:
        print("Error happened while reading GPS values")

def read_compass(master):
    try:
        print("Reading compass data...")
        while True:
            raw_imu_msg = master.recv_match(type='RAW_IMU', blocking=True, timeout=0.1)
            raw_imu = None

            if raw_imu_msg:
                data = raw_imu_msg.to_dict()
                xmag = data['xmag']
                ymag = data['ymag']

                rad = math.atan2(ymag, xmag)
                deg = math.degrees(rad)
                raw_imu = (deg + 360) % 360
                #print("read compass!!!")

                if raw_imu == None:
                    continue
                print("Giving degree : ", int(raw_imu))
                yield(raw_imu)

    except Exception as e:
        print(f"Error while reading compass data: {e}")
