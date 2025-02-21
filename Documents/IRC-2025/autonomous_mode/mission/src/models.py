from ultralytics import YOLO
import cv2
import yaml
import numpy as np
import torch

with open('config.yaml', 'r') as config:
    data = yaml.full_load(config)

#GLOBAL VARIABLES
CONE_CONFIDENCE_THRESHOLD = data.get("CONE_CONFIDENCE_THRESHOLD", 0.5)
CONE_DETECTION_MODEL_PATH = data.get("CONE_DETECTION_MODEL_PATH")
ARROW_DETECTION_MODEL_PATH = data.get("ARROW_DETECTION_MODEL_PATH")


class DetectionModel:
    def __init__(self, wanted: str):
        if wanted == "cone":
            self.model = YOLO(CONE_DETECTION_MODEL_PATH)
        else:
            self.model = YOLO(ARROW_DETECTION_MODEL_PATH)
        self.name = wanted
    
    def contains(self, frame):
        results = self.model(frame,conf=CONE_CONFIDENCE_THRESHOLD if self.name == "cone" else 0.25)

        if results[0].boxes is None or len(results[0].boxes) == 0:
            print(f"No {self.name} is detected!!")
            return False, frame
        else:
            print(f"{self.name} is detected!!")
            return True, results[0]
        
    def get_direction(self, image, box):
        if self.name == "arrow":
            x1,y1,x2,y2 = map(int, box.xyxy[0])
            print(x1,y1,x2,y2)
            image = image[y1:y2, x1:x2]

            # Preprocess the frame
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blurred, 50, 150)

            # Find contours
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL,  cv2.CHAIN_APPROX_SIMPLE)

            #Superimpose the contour over the frame
            contour_image = image.copy()
            cv2.drawContours(contour_image, contours, -1, (0, 255, 0), 3)
            
            # Preprocess the superimposed image
            gray = cv2.cvtColor(contour_image, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blurred, 50, 150)

            # Find contours for superimposed image
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contour_image = image.copy()
            cv2.drawContours(contour_image, contours, -1, (0, 255, 0), 3)

            # Process the largest contour (assumed to be the arrow)
            contour = max(contours, key=cv2.contourArea)

            # Fit an approximate polygon to the contour
            epsilon = 0.03 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            # print(approx)

            # If the arrow has a triangular shape (head) with a tail, we expect more than 3 points
            if len(approx) >= 4:
                # Identify the tip of the arrow (assumed to be the most prominent point)
                tip = approx[0][0] # The tip is often the first point in the approximation
                base = approx[1:4] # The base consists of the remaining points forming the tail
            
                # Calculate direction based on the tip and base
                # Use the first point (tip) and the last point of the base as a reference for the tail's orientation

                base_point = base[-1][0] # Tail's end point (can be approximated by the last base point)

                direction_vector = (tip[0] - base_point[0], tip[1] - base_point[1])

                # Calculate the angle of the direction vector
                angle = np.degrees(np.arctan2(direction_vector[1], direction_vector[0]))
                print(f"angle:{angle}")
                angle = angle + 90
                print(f"Arrow direction angle: {angle} degrees")

                direction = None
                if angle >=250 and angle <= 280:
                    direction = "left"
                elif angle >= 45 and angle < 135:
                    direction = "right"
                elif angle >= -135 and angle < -45:
                    direction = "left"
                print(f"Rover should move: {direction}")
            return direction
        else:
            return None
