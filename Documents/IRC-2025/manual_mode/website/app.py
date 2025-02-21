from flask import render_template, Response, Flask,request,jsonify
import random
import cv2
import numpy as np
# import pyzed.sl as sl
import time
import math
# from pymavlink import mavutil
import shutil
import os

app = Flask(__name__)

# zed = sl.Camera()
# left_image = sl.Mat()
# right_image = sl.Mat()
cap1 = cv2.VideoCapture("/dev/video0")
cap2 = cv2.VideoCapture("/dev/video2")  # Camera 2
cap3 = cv2.VideoCapture("/dev/video4")  # Camera 3
cap4 = cv2.VideoCapture("/dev/video6")  # Camera 3
# PIXHAWK_CONNECTION_PORT = "/dev/ttyACM2"
# PIXHAWK_BAUDRATE = 115200
# PIXHAWK = None
# PIXHAWK_stream_id = mavutil.mavlink.MAV_DATA_STREAM_ALL
# PIXHAWK_rate_hz = 10

cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 720)
cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# init_params = sl.InitParameters()
# init_params.camera_resolution = sl.RESOLUTION.VGA  # Use HD720 resolution
# init_params.depth_mode = sl.DEPTH_MODE.NONE          # Disable depth calculation
# init_params.coordinate_units = sl.UNIT.METER         # Set unit to meters

cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

cap3.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
cap3.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

cap4.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
cap4.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# Check if the cameras are opened correctly
# if not cap1.isOpened():
#     print("Error: Camera 1 not found.")

# status = zed.open(init_params)
# if status != sl.ERROR_CODE.SUCCESS:
#     print(f"Error opening zed camera : {status}")
#     exit(1)
if not cap2.isOpened():
    print("Error: Camera 2 not found.")
if not cap3.isOpened():
    print("Error: Camera 3 not found.")
if not cap4.isOpened():
    print("Error: Camera 4 not found.")

# try:
#     print(f"Connecting to Pixhawk on {PIXHAWK_CONNECTION_PORT}...")
#     PIXHAWK = mavutil.mavlink_connection(PIXHAWK_CONNECTION_PORT, baud=PIXHAWK_BAUDRATE)
#     PIXHAWK.wait_heartbeat()
#     print("Pixhawk connected! Heartbeat received.")
#     PIXHAWK.mav.request_data_stream_send(
#             PIXHAWK.target_system,
#             PIXHAWK.target_component,
#             PIXHAWK_stream_id,
#             PIXHAWK_rate_hz,
#             1
#     )
#     print(f"Requested data stream {PIXHAWK_stream_id} at {PIXHAWK_rate_hz} Hz.")
# except Exception as e:
#     print(f"Failed to connect to Pixhawk: {e}")

# def read_gps(master):
#     try:
#         print("Reading GPS Values")

#         while True:
#             gps_raw_int_msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=0.1)
#             gps_raw_int = None

#             if gps_raw_int_msg:
#                 data = gps_raw_int_msg.to_dict()
#                 lat = data['lat'] / 1e7
#                 lon = data['lon'] / 1e7
#                 alt = data['alt'] / 1e3
#                 gps_raw_int = [lat, lon, alt]
#                 #print("read gps!!!")
#                 return lon, lat
#     except Exception as e:
#         print("Error happened while reading GPS values")
#         return 0, 0
# def generate_frames1():
#     while True:
#         success, frame = cap1.read()  # Capture frame-by-frame
#         if not success:
#             break
#         else:
#             # Encode the frame as JPEG
#             ret, buffer = cv2.imencode('.jpg', frame)
#             frame = buffer.tobytes()
#             yield (b'--frame\r\n'
#                    b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

# def generate_zed_frame():
#     # Initialize the ZED Mat objects
#     left_image = sl.Mat()
#     right_image = sl.Mat()

#     while True:
#         # Grab frames from the ZED camera
#         if zed.grab() == sl.ERROR_CODE.SUCCESS:
#             # Retrieve the left and right images
#             zed.retrieve_image(left_image, sl.VIEW.LEFT)
#             zed.retrieve_image(right_image, sl.VIEW.RIGHT)

#             # Convert ZED Mat objects to NumPy arrays
#             left_frame = left_image.get_data()
#             right_frame = right_image.get_data()

#             # Concatenate the frames horizontally
#             combined_frame = np.hstack([left_frame, right_frame])

#             # Encode the combined frame as JPEG
#             ret, buffer = cv2.imencode('.jpg', combined_frame)
#             frame = buffer.tobytes()

#             # Yield the frame for streaming
#             yield (b'--frame\r\n'
#                    b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

def generate_frames2():
    while True:
        success, frame = cap2.read()  # Capture frame-by-frame
        if not success:
            break
        else:
            # Encode the frame as JPEG
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')


def generate_frames3():
    while True:
        success, frame = cap3.read()  # Capture frame-by-frame
        if not success:
            break
        else:
            # Encode the frame as JPEG
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

def generate_frames4():
    while True:
        success, frame = cap4.read()  # Capture frame-by-frame
        if not success:
            break
        else:
            # Encode the frame as JPEG
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def index():
    return render_template('dashboard.html')

@app.route('/video_feed1')
def video_feed1():
    # Route for the video feed
    print("Inside feed1")
    return Response(generate_zed_frame(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video_feed2')
def video_feed2():
    # Route for the video feed
    return Response(generate_frames2(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video_feed3')
def video_feed3():
    # Route for the video feed
    return Response(generate_frames3(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/video_feed4')
def video_feed4():
    # Route for the video feed
    return Response(generate_frames4(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/science-subsystem')
def science_subsystem():
    data = {
        "MQ_8": {"value": 0},
        "MQ_9": {"percentage": 0, "stroke_dasharray": f"{0}, {100}"},
        "MQ_135": {"value": 0},
        "BME680": {"temperature": 0, "humidity": 0, "pressure": 0, "voc": 0},
        "SOIL_MOISTURE": {"value": 0, "stroke_dasharray": f"{0}, {100}"},
        "SOIL_TEMPERATURE": 0,
        "pH_Level": 0,
        "NPK_SENSOR": {"nitrogen": 0, "phosphorus": 0, "potassium": 0}
    }
    return render_template('science.html', data=data)

@app.route('/power-subsystem')
def power_subsystem():
    return render_template('power.html')

@app.route('/autonomous-mode/GNSS-submitted')
def autonomous_mode():
    return render_template('autonomous.html')

@app.route('/autonomous-mode')
def autonomous_GPS():
    return render_template('GNSSsubmission.html')

@app.route("/save", methods=["POST"])
def save_coordinates():
    try:
        data = request.get_json()

        # Folder where coordinates will be stored
        folder = os.getcwd()+"\static\GNSS_coordinates"
        print(folder)

        # Delete the folder if it exists, then create a new one
        if os.path.exists(folder):
            shutil.rmtree(folder)
        os.makedirs(folder)

        # Extract coordinates from the JSON payload
        coords = [
            data.get("gnss1", ""),
            data.get("gnss2", ""),
            data.get("gnss3", ""),
            data.get("gnss4", ""),
            data.get("gnss5", ""),
            data.get("gnss6", ""),
            data.get("gnss7", "")
        ]
        print(coords)
        content = "\n".join(coords)

        # Save the coordinates into a text file inside the folder
        file_path = os.path.join(folder, "coordinates.txt")
        with open(file_path, "w", encoding="utf-8") as f:
            f.write(content)

        # Return a JSON response with a redirect URL
        # Replace the URL below with your desired destination
        return jsonify({"redirectUrl": "/autonomous-mode/GNSS-submitted"})

    except Exception as e:
        print("Error processing GNSS coordinates:", e)
        return jsonify({"error": "Internal Server Error"}), 500

@app.route('/science-subsystem/hydrogen-concentration')
def hydrogen_concentration_data():
    val = random.randint(100, 10000)
    return {"hydrogen_concentration": val}

@app.route('/science-subsystem/combustible-gases')
def combustible_gases_data():
    val = random.randint(0, 100)
    return {"combustible_gases": val}

@app.route('/science-subsystem/pH-Level')
def pH_Level_data():
    val = random.randint(0, 14)
    return {"pH_Level": val}

@app.route('/science-subsystem/air-quality')
def air_quality_data():
    val = random.randint(10, 1000)
    return {"air_quality": val}

@app.route('/science-subsystem/soil-moisture')
def soil_moisture_data():
    val = random.randint(0, 100)
    return {"soil_moisture": val}

@app.route('/science-subsystem/soil-temperature')
def soil_temperature_data():
    val = random.randint(-40, 125)
    return {"soil_temperature": val}

@app.route('/science-subsystem/NPK')
def NPK_data():
    NPK = {"nitrogen": random.randint(0, 1000), "phosphorus": random.randint(0, 100), "potassium": random.randint(0, 1000)}
    return NPK

@app.route('/science-subsystem/BME680')
def BME680_data():
    BME680 = {"temperature": random.randint(-40, 85), "humidity": random.randint(0, 100), "pressure": random.randint(30000, 110000), "voc": random.randint(0,500)}
    return BME680

@app.route('/GPS')
def rover_location():
    lat,lon = read_gps(PIXHAWK)
    location = {
        'latitude': lat,
        'longitude': lon
    }
    return location



if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000,debug=True)
