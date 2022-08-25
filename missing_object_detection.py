import cv2
import time

def missing_object_detection():

    cap = cv2.VideoCapture(6)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)


    while True:

        ret, frame = cap.read()

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        height, width , ret = frame.shape

        #cx = int(width /2)
        #cy = int(height /2)
        cx_1 = int(485) #spring
        cy_1 = int(505)

        cx_2 = int(479) # cap
        cy_2 = int(425)

        cx_3 = int(596) #head
        cy_3 = int(470)

        cx_4 = int(844) #ink
        cy_4 = int(420)

        cx_5 = int(806) #body
        cy_5 = int(536)




        pixel_center_1 = hsv_frame[cy_1, cx_1]
        pixel_center_2 = hsv_frame[cy_2, cx_2]
        pixel_center_3 = hsv_frame[cy_3, cx_3]
        pixel_center_4 = hsv_frame[cy_4, cx_4]
        pixel_center_5 = hsv_frame[cy_5, cx_5]



        hue_value_1 = pixel_center_1[0]
        hue_value_1_s = pixel_center_1[1]
        hue_value_2 = pixel_center_2[0]
        hue_value_2_s = pixel_center_2[1]
        hue_value_3 = pixel_center_3[0]
        hue_value_3_s = pixel_center_3[1]
        hue_value_3_v = pixel_center_3[2]
        hue_value_4 = pixel_center_4[0]
        hue_value_4_s = pixel_center_4[1]
        hue_value_4_v = pixel_center_4[2]
        hue_value_5 = pixel_center_5[0]
        hue_value_5_s = pixel_center_5[1]
        hue_value_5_v = pixel_center_5[2]

        #print(hue_value_1_s)
        
        #print("The H, S, V values of spring:")
        #print(pixel_center_1) # [H,S,V]
        #print("The H, S, V values of cap:")
        #print(pixel_center_2) # [H,S,V]
        #print("The H, S, V values of head:")
        #print(pixel_center_3) # [H,S,V]
        #print("The H, S, V values of ink:")
        #print(pixel_center_4) # [H,S,V]
        #print("The H, S, V values of body:")
        #print(pixel_center_5) # [H,S,V]
        time.sleep(0.2)
        color = "Undefined"
        
        if hue_value_1_s > 100 :
            print("Missing spring")
        else:
            pass
            #print("Spring is here")
        
        if hue_value_2_s > 180 :
            print("Missing cap")
        else:
            pass
            #print("Cap is here")
        
        if hue_value_3_v < 130 and hue_value_3_s < 90 :
            print("Missing head")
        else:
            pass
            #print("Head is here")
        
        if hue_value_4_v < 170 :
            print("Missing ink")
        else:
            pass
            #print("ink is here")

        
        if hue_value_5_v < 130 and hue_value_5_s < 50 :
            print("Missing body")
        else:
            pass
            #print("body is here")
        
        



        pixel_center_bgr = frame[cy_1, cx_1]
        b ,g, r =int(pixel_center_bgr[0]) ,int(pixel_center_bgr[1]) ,int(pixel_center_bgr[2]) 



        cv2.circle(frame, (cx_1, cy_1), 5, (0, 0, 255), 2)
        cv2.circle(frame, (cx_2, cy_2), 5, (0, 0, 255), 2)
        cv2.circle(frame, (cx_3, cy_3), 5, (0, 0, 255), 2)
        cv2.circle(frame, (cx_4, cy_4), 5, (0, 0, 255), 2)
        cv2.circle(frame, (cx_5, cy_5), 5, (0, 0, 255), 2)




        cv2.imshow('frame',frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        




    cap.release()
    cv2.destroyAllWindows()

missing_object_detection()