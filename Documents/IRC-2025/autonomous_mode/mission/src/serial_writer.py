import serial
import yaml

#Read the config file
with open('config.yaml', 'r') as config:
    data = yaml.full_load(config)


#GLOBAL VARIABLES
WHEELS_PORT = data.get("WHEELS_PORT")
WHEELS_BAUDRATE = data.get("WHEELS_BAUDRATE")

class SerialObject():
    def __init__(self):
        self.port = WHEELS_PORT
        self.baudrate = WHEELS_BAUDRATE
        self.object = ...
        try:
            self.object = serial.Serial(self.port, self.baudrate, timeout=1.0)
        except Exception as e:
            self.object = None
            print("Error happened while setting up the port", e)
    
    def write(self, left_speed, right_speed):
        message = f"l{right_speed},r{left_speed}\n".encode()
        self.object.write(message)
        return True
