import cv2
import numpy as np

def get_direction(self, image, box):
    if self.name == "arrow":
        x1,y1,x2,y2 = map(int, box.xyxy[0])
        print(x1,y1,x2,y2)
        image = image[y1:y2, x1:x2]
        
        # Load and preprocess the image
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)

        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL,  cv2.CHAIN_APPROX_SIMPLE)
        contour_image = image.copy()
        cv2.drawContours(contour_image, contours, -1, (0, 255, 0), 3)

        gray = cv2.cvtColor(contour_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)

        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contour_image = image.copy()
        cv2.drawContours(contour_image, contours, -1, (0, 255, 0), 3)

        # Process the largest contour (assumed to be the arrow)
        contour = max(contours, key=cv2.contourArea)

        # Fit an approximate polygon to the contour
        epsilon = 0.03 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        print(approx)

        # If the arrow has a triangular shape (head) with a tail, we expect more than 3 points
        if len(approx) >= 4:
            # Identify the tip of the arrow (assumed to be the most prominent point)
            tip = approx[0][0] # The tip is often the first point in the approximation
            base = approx[1:4] # The base consists of the remaining points forming the tail

            # Draw the contour for visualization
            cv2.drawContours(image, [approx], 0, (0, 255, 0), 3)
        
            # Calculate direction based on the tip and base

            # Use the first point (tip) and the last point of the base as a reference for the tail's orientation

            base_point = base[-1][0] # Tail's end point (can be approximated by the last base point)

            direction_vector = (tip[0] - base_point[0], tip[1] - base_point[1])

            # Calculate the angle of the direction vector
            angle = np.degrees(np.arctan2(direction_vector[1], direction_vector[0]))
            angle = angle + 90
            print(f"Arrow direction angle: {angle} degrees")

            if angle >= 45 and angle < 135:
                direction = "right"
            elif angle >= -135 and angle < -45:
                direction = "left"


            print(f"Rover should: {direction}")
            return direction
    else:
        return None
            
