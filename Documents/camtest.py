import cv2

# Open three video capture objects for each camera (usually /dev/video0, /dev/video1, /dev/video2, etc.)
cap1 = cv2.VideoCapture("/dev/video0")  # Camera 1
cap2 = cv2.VideoCapture("/dev/video2")  # Camera 2
cap3 = cv2.VideoCapture("/dev/video4")  # Camera 3
cap4 = cv2.VideoCapture("/dev/video6")  # Camera 3

cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

cap3.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
cap3.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

cap4.set(cv2.CAP_PROP_FRAME_WIDTH, 480)
cap4.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

# Check if the cameras are opened correctly
if not cap1.isOpened():
    print("Error: Camera 1 not found.")
if not cap2.isOpened():
    print("Error: Camera 2 not found.")
if not cap3.isOpened():
    print("Error: Camera 3 not found.")
if not cap4.isOpened():
    print("Error: Camera 4 not found.")

# Loop to capture frames from the cameras
while True:
    ret1, frame1 = cap1.read()
    ret2, frame2 = cap2.read()
    ret3, frame3 = cap3.read()
    ret4, frame4 = cap4.read()

    if ret1:
        cv2.imshow("Camera 1", frame1)

    if ret2:
        cv2.imshow("Camera 2", frame2)

    if ret3:
        cv2.imshow("Camera 3", frame3)
    
    if ret4:
        cv2.imshow("Camera ", frame4)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera objects and close all OpenCV windows
cap1.release()
cap2.release()
cap3.release()
cv2.destroyAllWindows()
