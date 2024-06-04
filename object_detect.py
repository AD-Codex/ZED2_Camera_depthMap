#!/usr/bin/env python3

# # object detection node
# # persons and cars
# # publish /objects/objects_coords array
# # 1d array pub - [ cls number, object count, min_point, min_p_X, min_p_Y, S_point, S_p_X, S_p_Y, E_point, E_p_X, E_p_Y]

import rospy
import cv2
from ultralytics import YOLO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
import numpy as np

shape = (376, 672, 3)
frame_cap = np.zeros(shape)
shape = (376, 672)
depth_cap = np.zeros(shape)

model = YOLO('yolov8n-seg.pt')
frame_success = False
depth_success = False




def image_read(msg: Image):
    global frame_cap
    global frame_success

    bridge = CvBridge()
    orig = bridge.imgmsg_to_cv2(msg, "bgr8")
    frame_cap = orig
    frame_success = True


def depth_read(msg: Image):
    global depth_cap
    global depth_success

    bridge = CvBridge()
    orig = bridge.imgmsg_to_cv2(msg, "32FC1")
    depth_cap = orig
    depth_success = True


def publish_objectImage(img):
    global pub_objectImage

    bridge = CvBridge()
    imgMsg = bridge.cv2_to_imgmsg(img,"bgr8")
    pub_objectImage.publish(imgMsg)



if __name__ == '__main__':
    rospy.init_node('object_detect_node')
    rospy.loginfo("object identify node start")

    rate = rospy.Rate(30)

    sub_image = rospy.Subscriber("/zed_node/image", Image, callback=image_read)
    sub_depth = rospy.Subscriber("/zed_node/depth", Image, callback=depth_read)
    pub_coord = rospy.Publisher("/objects/objects_coords", Float64MultiArray, queue_size=50)
    pub_objectImage = rospy.Publisher('/objects/objects_image', Image, queue_size=10)

    while not rospy.is_shutdown():

        if ( frame_success ):
            # # convert (376, 672, 3) to (376, 640, 3)
            # remove first and last 16 columns
            frame = frame_cap[:, +16:-16, :]
            # cv2.imshow("upgrade frame", frame)
    
            # # YOLO model load ---------------------------------------------------------------
            results = model(frame)

            # person, vehicle mask data
            object_mask = []

            try:
                for r in results:
                    masks = r.masks.cuda()
                    masks_xy = masks.xy
                    boxes = r.boxes.cuda()
                    object_XY = boxes.xyxy

                    person_count_val = -1
                    vehicl_count_val = -1

                    for count, dot_array in enumerate(masks_xy):
                        detect_mask = []

                        start_point_x = int(object_XY[count][0])
                        start_point_y = int(object_XY[count][1])
                        end_point_x = int(object_XY[count][2])
                        end_point_y = int(object_XY[count][3])

                        start_point = ( start_point_x, start_point_y)
                        end_point = ( end_point_x, end_point_y)

                        print("detect cls: ", int(r.boxes.cls[count]), " name: ", r.names[int(r.boxes.cls[count])])

                        if ( int(r.boxes.cls[count]) == 0):
                            person_count_val = person_count_val +1
                            # print(masks.data.size())
                            # print(masks.data[count])

                            # # person mask bitmap data -----------------------------------------------------
                            detect_mask.append(int(r.boxes.cls[count]))
                            detect_mask.append(person_count_val +1)
                            detect_mask.append(results[0].masks.data[count].cpu().numpy())
                            detect_mask.append(start_point)
                            detect_mask.append(end_point)
                            object_mask.append(detect_mask)
                        elif ( int(r.boxes.cls[count]) == 2 or int(r.boxes.cls[count]) == 3 or int(r.boxes.cls[count]) == 5 or int(r.boxes.cls[count]) == 7):
                            vehicl_count_val = vehicl_count_val +1
                            # # vehicle mask bitmap data -----------------------------------------------------
                            detect_mask.append(int(r.boxes.cls[count]))
                            detect_mask.append(vehicl_count_val +1)
                            detect_mask.append(results[0].masks.data[count].cpu().numpy())
                            detect_mask.append(start_point)
                            detect_mask.append(end_point)
                            object_mask.append(detect_mask)
                            
                print("detection count ", len(object_mask))
                print("------------------------------------------------------------")
            except:
                print("model detection error")

            # # YOLO detection end ----------------------------------------------------------------

            
            # # depth data processing ------------------------------------------------------------
            try:
                publish_coord = []
                object_coord = []

                for detect_mask in object_mask:
                    
                    frame = cv2.rectangle(frame, detect_mask[3], detect_mask[4], (255,0,0), 1)
                    frame = cv2.putText( frame, str(r.names[detect_mask[0]]), detect_mask[3], cv2.FONT_HERSHEY_SIMPLEX, 1, (50,50,255), 1, cv2.LINE_AA)
                    

                    # # mask boundary coords (image_mask X depth_matrix) ---------------------------
                    # get detect person and remove first 8 rows to 376x640
                    object_frame = detect_mask[2][ +4:-4, :]
                    person_depth_array = np.array(object_frame) * np.array(depth_cap[:,+16:-16])
                    # print("person_depth_array", person_depth_array.shape)


                    # # minimum point of mask -----------------------------------------------
                    min_array= np.empty((0, 2), dtype=float)
                    person_x_min=0
                    person_y_min=0
                    for row in person_depth_array:
                        # print(type(row))
                        row[ row == 0 ] = np.nan
                        # print("max", np.nanmax(row) , "min", np.nanmin(row))
                        if np.any(~np.isnan(row)):
                            person_x_min = np.nanargmin(np.abs(row))
                            min_array = np.vstack((min_array, np.array([np.nanmin(np.abs(row)) , person_x_min])))
                        else:
                            min_array = np.vstack((min_array, np.array([np.nan , np.nan])))
                        
                    # print(min_array.shape)
                    # # minimum value select
                    minimum_value = np.nanmin(min_array[:,0])
                    person_y_min = np.nanargmin(min_array[:,0])
                    person_x_min = min_array[person_y_min,1]
                    # print('minimun', minimum_value, "at:", person_x_min, person_y_min)

                    frame = cv2.circle( frame , (int(person_x_min),int(person_y_min)), 1, (255,0,0), 5)
                    frame = cv2.putText( frame, str(int(minimum_value)), [int(person_x_min),int(person_y_min)], cv2.FONT_HERSHEY_SIMPLEX, 1, (50,50,255), 2, cv2.LINE_AA)
                    

                    # # boundary point values -----------------------------------------------------
                    start_point_x = detect_mask[3][0]
                    start_point_y = detect_mask[3][1]
                    end_point_x = detect_mask[4][0]
                    end_point_y = detect_mask[4][1]
                    # # box1 = [start_point_x,start_point_y], [start_point_x + 40, end_point_y]
                    # # box2 = [end_point_x - 40,start_point_y] , [end_point_x, end_point_y]
                    S_point_min = np.nanmin(np.abs(person_depth_array[ int(start_point_y):int(end_point_y), int(start_point_x):int(start_point_x+40) ]))
                    E_point_min = np.nanmin(np.abs(person_depth_array[ int(start_point_y):int(end_point_y), int(end_point_x-40):int(end_point_x) ]))
                    # print("start point mini :", S_point_min)
                    # print("end point min :", E_point_min)

                    frame = cv2.putText( frame, str(int(S_point_min)), ( start_point_x, end_point_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (50,50,255), 2, cv2.LINE_AA)
                    frame = cv2.putText( frame, str(int(E_point_min)), ( end_point_x, end_point_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (50,50,255), 2, cv2.LINE_AA)
                    

                    # # publish depth data -------------------------------------------------------------
                    object_coord.append(detect_mask[0])
                    object_coord.append(detect_mask[1])

                    object_coord.append(minimum_value)          # min point
                    object_coord.append(person_x_min)
                    object_coord.append(person_y_min)

                    object_coord.append(S_point_min)            # start point
                    object_coord.append(start_point_x)
                    object_coord.append(start_point_y)

                    object_coord.append(E_point_min)            # end point
                    object_coord.append(end_point_x)
                    object_coord.append(end_point_y)
                    # print(object_coord)

                    # # depth frame display
                    # frame_name = "object_depth" + str(detect_mask[1])
                    # cv2.imshow(frame_name, person_depth_array/5000)
                
                print(object_coord)
                publish_coord = Float64MultiArray()
                publish_coord.data = object_coord
                pub_coord.publish(publish_coord)

                # # frame with masks
                publish_objectImage(frame)
                cv2.imshow("frame data", frame)

                print("------------------------------------------------------------")
            except:
                print("depth processing error")


        if cv2.waitKey(1) & 0XFF == ord("q"):
            break

        frame_success = False

        rate.sleep()
    

    rospy.spin()