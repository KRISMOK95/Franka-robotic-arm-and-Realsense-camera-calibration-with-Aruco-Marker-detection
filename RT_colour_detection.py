import cv2
import time
from frankapy import FrankaArm


REALSENSE_INTRINSICS = "/home/kalong/Documents/camera-calibration/calib/realsense_intrinsics.intr"
REALSENSE_EE_TF = "/home/kalong/Documents/camera-calibration/calib/realsense_ee.tf"

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--intrinsics_file_path", type=str, default=REALSENSE_INTRINSICS
    )
    parser.add_argument("--extrinsics_file_path", type=str, default=REALSENSE_EE_TF)
    args = parser.parse_args()

    print("Starting robot")
    fa = FrankaArm()

    cap = cv2.VideoCapture(6)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)


    name = input("Enter the expected color from container: ( R , B , G , Y , W )")
    name = name.lower()
    print("The color is: " , name)

    while True:

        ret, frame = cap.read()

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        height, width , ret = frame.shape

        cx = int(width /2)
        cy = int(height /2)
        pixel_center = hsv_frame[cy, cx]
        hue_value = pixel_center[0]
        hue_value_s = pixel_center[1]
        hue_value_v = pixel_center[2]
        print("The H, S, V values:")
        print(pixel_center) # [H,S,V]

        color = "Undefined"

        # red = [0,210, 135]
        # white =[25, 25, 138]


        if hue_value < 5:
            color =  "RED"
            color_name = "r"
        elif hue_value < 20:
            color = "WHITE"
            color_name = "w"
        elif hue_value < 40 :
            color = "YELLOW"
            color_name = "y"
        elif hue_value < 78 :
            color = "GREEN"
            color_name = "g"
        elif hue_value < 131:
            color = "BLUE"
            color_name = "b"
        else:
            color = "POINT TO COLOR"

        pixel_center_bgr = frame[cy , cx]
        b ,g, r =int(pixel_center_bgr[0]) ,int(pixel_center_bgr[1]) ,int(pixel_center_bgr[2]) 
        cv2.rectangle(frame, (cx - 220, 10), (cx + 200, 120), (255, 255, 255), -1)
        cv2.putText(frame, color, (cx - 200, 100), 0, 3, (b, g, r), 5)
        cv2.circle(frame, (cx, cy), 5, (25, 25, 25), 3)




        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        if name != color_name:
            print("Wrong color")
            fa.stop_skill()
            fa.reset_pose()
            fa.reset_joints()
            print("Please place the ", name, " color into the container")
            break
        else:
            print("Correct color")
        




    cap.release()
    cv2.destroyAllWindows()