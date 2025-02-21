import matplotlib.pyplot as plt
import time
import tkinter as tk
from tkinter import simpledialog
import requests

#Map config
plt.ion()
fig, ax = plt.subplots(figsize=(8,8))
ax.set_xlim(-10,10) # longitude
ax.set_ylim(-10,10) # latitude
ax.set_title("Rover navigation path")
ax.set_xlabel("Longitude")
ax.set_ylabel("Latitude")

#helper functions
def get_destination_coordinates():
    root = tk.Tk()
    root.withdraw()

    lat = simpledialog.askfloat("Input", "Enter the destination latitude:")
    lon = simpledialog.askfloat("Input", "Enter the destination longitude:")

    root.destroy()
    return lon, lat

def get_current_location():
    GPS_URL = "http://192.168.1.111:5000/GPS" # NEED TO WRITE AN ENDPOINT USING FLASK IN ROVER

    try:
        response = requests.get(GPS_URL)

        response.raise_for_status()

        data = response.json()

        lon = data.get("longitude")
        lat = data.get("latitude")

        print("current location : " + str(lon) + "," + str(lat))

        return lon, lat
    except Exception as e:
        print(f"Error Occured {e}")
        return None, None

# Data
rover_path = []
destination = get_destination_coordinates()

#plot destination
if destination:
    ax.plot(destination[0], destination[1], 'ro', label='Destination')
    ax.legend()

increment = 0.1

while True:

    #simulated GPS data
    current_location = get_current_location()
    rover_path.append(current_location)

    #update plot
    if current_location:
        lons, lats = zip(*rover_path)
        ax.plot(lons, lats, '-b', label="Path" if len(rover_path) == 1 else "")
    else:
        print("Corrupted GPS value from the endpoint")

    plt.pause(0.1)
    time.sleep(0.5)

    increment += 0.1

plt.ioff()
plt.show()
