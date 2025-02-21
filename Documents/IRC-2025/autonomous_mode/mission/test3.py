import sys
import pyzed.sl as sl

def get_camera_orientation():
    # Create a ZED Camera object
    zed = sl.Camera()

    # Initialize the camera
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD1080
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA

    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Error opening the camera")
        exit(1)

    # Create a Pose object
    pose = sl.Pose()

    # Retrieve the camera pose
    runtime_parameters = sl.RuntimeParameters()
    while True:
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Get the camera pose
            zed.get_pose(pose, sl.REFERENCE_FRAME.WORLD)

            # Get the rotation matrix (orientation)
            rotation = pose.get_rotation()
            orientation = rotation.get_matrix()

            print("Camera Orientation (Rotation Matrix):")
            print(orientation)
            
            # You can also get Euler angles if needed
            euler_angles = pose.get_euler_angles()
            print("Camera Euler Angles (roll, pitch, yaw):")
            print(euler_angles)

if __name__ == "__main__":
    get_camera_orientation()
