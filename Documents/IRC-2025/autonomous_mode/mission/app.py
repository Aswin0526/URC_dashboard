"""
starting point for the autonomous navigation task for IRC 2025
"""
import time
import yaml
import cv2
from src.rosnodes import start_node
from src.serial_writer import SerialObject
from src.models import DetectionModel
from src.navigation_helper import is_boxnear, adjust_direction
from pymavlink import mavutil
from src.position import read_compass, read_gps

#Read the config file
with open('config.yaml', 'r') as config:
    data = yaml.full_load(config)

#GLOBAL VARIABLES
WHEELS_CONTROLLER = SerialObject()
ARROW_MODEL = DetectionModel("arrow")
CONE_MODEL = DetectionModel("cone")
ZED_SUBSCRIBER = start_node()
THRESHOLD_DEGREE = data.get("THRESHOLD_DEGREE")
PIXHAWK_CONNECTION_PORT = data.get("PIXHAWK_CONNECTION_PORT")
PIXHAWK_BAUDRATE = data.get("PIXHAWK_BAUDRATE")
PIXHAWK = None
PIXHAWK_stream_id = mavutil.mavlink.MAV_DATA_STREAM_ALL
PIXHAWK_rate_hz = 10
SPEED = 15
ALIGN_SPEED = 8
DIRECTION = None
time.sleep(2)

try:
    print(f"Connecting to Pixhawk on {PIXHAWK_CONNECTION_PORT}...")
    PIXHAWK = mavutil.mavlink_connection(PIXHAWK_CONNECTION_PORT, baud=PIXHAWK_BAUDRATE)
    PIXHAWK.wait_heartbeat()
    print("Pixhawk connected! Heartbeat received.")
    PIXHAWK.mav.request_data_stream_send(
            PIXHAWK.target_system,
            PIXHAWK.target_component,
            PIXHAWK_stream_id,
            PIXHAWK_rate_hz,
            1
    )
    print(f"Requested data stream {PIXHAWK_stream_id} at {PIXHAWK_rate_hz} Hz.")
except Exception as e:
    print(f"Failed to connect to Pixhawk: {e}")

# Normalize angles between 0 and 360
def normalize_angle(angle):
    return (angle + 360) % 360

def left_turn():
    global DIRECTION
    compass = read_compass(PIXHAWK)
    initial_degree = next(compass)
    desired_degree = (initial_degree + 90) % 360
    print(f"Initial degree: {initial_degree}, Desired degree: {desired_degree}")

    # Adjust the direction until the desired heading is reached
    while True:
        current_degree = next(compass)
        print(f"Current degree: {current_degree}")
        if abs(current_degree - desired_degree) <= THRESHOLD_DEGREE:
            WHEELS_CONTROLLER.write(0,0)
            break
        WHEELS_CONTROLLER.write(ALIGN_SPEED, -ALIGN_SPEED)

    WHEELS_CONTROLLER.write(0, 0)  # Stop the rover
    print("Rover turned left")
    DIRECTION = None
    return True

def right_turn():
    global DIRECTION
    compass = read_compass(PIXHAWK)
    initial_degree = next(compass)
    desired_degree = (initial_degree + 270) % 360  # Right turn is 90 degrees
    print(f"Initial degree: {initial_degree}, Desired degree: {desired_degree}")

    # Adjust the direction until the desired heading is reached
    while True:
        current_degree = next(compass)
        print(f"Current degree: {current_degree}")
        
        if abs(current_degree - desired_degree) <= THRESHOLD_DEGREE:
            WHEELS_CONTROLLER.write(0,0)
            break

        WHEELS_CONTROLLER.write(-ALIGN_SPEED, ALIGN_SPEED)

    WHEELS_CONTROLLER.write(0, 0)  # Stop the rover
    print("Rover turned right")
    DIRECTION = None
    return True

try:
    while True:
        frame, depth = ZED_SUBSCRIBER.image, ZED_SUBSCRIBER.depth_img
        try:
            if frame.any():
                # cv2.imshow("feed",frame)

                cone_ret, cone_result = CONE_MODEL.contains(frame)
                
                if cone_ret:
                    # cv2.imshow("Feed", cone_result.plot())
                    print("Inside cone_ret")
                    
                    if is_boxnear(cone_result.boxes[0], depth):
                        print("cone is near to the rover")
                        WHEELS_CONTROLLER.write(0, 0)
                        quit()
                    
                    align_direction = adjust_direction(cone_result.boxes[0])
                    print(f"alignment direction is {align_direction}")
                    if align_direction == "left":
                        print(f"inside {align_direction}")
                        WHEELS_CONTROLLER.write(ALIGN_SPEED, -ALIGN_SPEED)
                    elif align_direction == "right":
                        print(f"inside {align_direction}")
                        WHEELS_CONTROLLER.write(-ALIGN_SPEED, ALIGN_SPEED)                
                    else:
                        print(f"inside forward")
                        WHEELS_CONTROLLER.write(ALIGN_SPEED,ALIGN_SPEED)
                    continue

                arrow_ret, arrow_result = ARROW_MODEL.contains(frame)

                if arrow_ret:
                    # cv2.imshow("Feed", arrow_result.plot())
                    print("Inside arrow_ret")
                    arrow_direction = ARROW_MODEL.get_direction(frame, arrow_result.boxes)

                    if arrow_direction != None:
                        print(f"arrow direction is {arrow_direction}")
                        DIRECTION = arrow_direction
                        print(f"Direction set to {DIRECTION}")

                    if is_boxnear(arrow_result.boxes[0], depth):
                        print("arrow is near to the rover")
                        WHEELS_CONTROLLER.write(0, 0)
                        time.sleep(10)

                        if arrow_direction == "left":
                            left_turn()
                        if arrow_direction == "right":
                            right_turn()
                        continue
                    
                    align_direction = adjust_direction(arrow_result.boxes[0])

                    print(f"alignment direction is {align_direction}")
                    if align_direction == "left":
                        print(f"inside {align_direction}")
                        WHEELS_CONTROLLER.write(ALIGN_SPEED, -ALIGN_SPEED)
                    elif align_direction == "right":
                        print(f"inside {align_direction}")
                        WHEELS_CONTROLLER.write(-ALIGN_SPEED, ALIGN_SPEED)                
                    else:
                        print(f"inside forward")
                        WHEELS_CONTROLLER.write(ALIGN_SPEED, ALIGN_SPEED)
                    continue

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                WHEELS_CONTROLLER.write(SPEED,SPEED)

            else:
                print("Couldn't capture the picture!!!")
        except Exception as e:
            print("Error in image processing: " + str(e))
except KeyboardInterrupt:
    print("You stopped the program")
    quit()
except Exception as e:
    print("Error in main flow: " + str(e))
finally:
    if WHEELS_CONTROLLER.object:
        WHEELS_CONTROLLER.object.close()
    cv2.destroyAllWindows()
