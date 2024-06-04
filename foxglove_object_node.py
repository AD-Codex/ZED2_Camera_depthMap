#!/usr/bin/env python3

# # width 640, height 376
# # person data list, y=min+point, 

# # https://automaticaddison.com/coordinate-frames-and-transforms-for-ros-based-mobile-robots/
# # https://foxglove.dev/blog/understanding-ros-transforms
# # http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20broadcaster%20%28Python%29



import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
from tf.transformations import *
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import numpy as np
import math



person_coords = []
old_personCount = 0
vehicle_coords = []
coord_resived = False

marker = Marker()
markerArray = MarkerArray()


# # Marker initialization
def marker_init():
    global marker

    marker.header.frame_id = "base_link"
    marker.header.stamp = rospy.Time.now()

    marker.type = Marker.CYLINDER
    marker.id = 0
    marker.action = 0

    # Scale
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1

    # Color
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 1.0

    # Pose
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0

    # matrix shift
    q_orig = quaternion_from_euler(0, 0, 0)
    q_rot = quaternion_from_euler(3.14159, 0, 0)
    q_new = quaternion_multiply(q_rot, q_orig)
    # print(q_new)

    marker.pose.orientation.x = q_new[0]
    marker.pose.orientation.y = q_new[1]
    marker.pose.orientation.z = q_new[2]
    marker.pose.orientation.w = q_new[3] 




# # coordinate read
def coord_read(msg: Float64MultiArray):
    global person_coords
    global vehicle_coords
    global coord_resived

    coord_list = msg.data
    person_coords = []
    vehicle_coords = []
    coord_resived = True

    for i in range(len(coord_list)%10):
        strat_point = i*11
        if (coord_list[strat_point] == 0):
            # print("person")
            person_coords.append(coord_list[strat_point:11+strat_point])
        elif (coord_list[strat_point] == 2):
            # print("car")
            vehicle_coords.append(coord_list[strat_point:11+strat_point])
        elif (coord_list[strat_point] == 3):
            # print("car")
            vehicle_coords.append(coord_list[strat_point:11+strat_point])
        elif (coord_list[strat_point] == 5):
            # print("car")
            vehicle_coords.append(coord_list[strat_point:11+strat_point])
        elif (coord_list[strat_point] == 7):
            # print("car")
            vehicle_coords.append(coord_list[strat_point:11+strat_point])
    
    print(person_coords, vehicle_coords)



# # get exact X Y Z
def yxz(point, p_X, p_Y):
    x_theta = math.atan( 320/(abs(p_X)*1.732))
    z_theta = math.atan( 320/(abs(p_Y)*1.732))
    y = point * math.sin( x_theta)
    x = point * math.cos( x_theta)
    z = - point * math.cos( z_theta)
    if ( p_X <= 0):
        x = -x
    if ( p_Y <= 0):
        z = -z

    # print(y, x, z)
    return y,x,z
    

# # update markers
def pointUpdate():
    global person_coords
    global old_personCount
    global vehicle_coords
    global marker
    global markerArray

    try:
        # # person limit cycle
        for i in range(len(person_coords)):
            min_point = person_coords[i][2]
            min_p_X = person_coords[i][3] - 320
            min_p_Y = person_coords[i][4] - 188
            S_point = person_coords[i][5]
            S_p_X = person_coords[i][6] - 320
            S_p_Y = person_coords[i][7] -188
            E_point = person_coords[i][8]
            E_p_X = person_coords[i][9] - 320
            E_p_Y = person_coords[i][10] -188

            y1, x1, z1 = yxz(min_point, min_p_X, min_p_Y)
            y2, x2, z2 = yxz(S_point, S_p_X, S_p_Y)
            y3, x3, z3 = yxz(E_point, E_p_X, E_p_Y)

            y = (y2 + y3)/2
            x = (x2 + x3)/2
            scale = (((y2-y3)**2 + (x2-x3)**2)**(1/2) )/2
            # print(scale)

            marker.id = i

            marker.color.r = 0.0 + i/10
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0

            marker.scale.x = scale/500
            marker.scale.y = scale/500
            marker.scale.z = 1

            marker.pose.position.x = x/500
            marker.pose.position.y = y/500
            marker.pose.position.z = 0.4

            markerArray.markers.append(marker)

        if ( len(person_coords) < old_personCount):
            for i in range(old_personCount - len(person_coords)):
                marker.scale.x = 0
                marker.scale.y = 0
                marker.scale.z = 0
                marker.id = i + len(person_coords)
                markerArray.markers.append(marker)

        old_personCount = len(person_coords)

        # # vehicle limit cycle
        for i in range(len(vehicle_coords)):
            min_point = vehicle_coords[i][2]
            min_p_X = vehicle_coords[i][3] - 320
            min_p_Y = vehicle_coords[i][4] - 188
            S_point = vehicle_coords[i][5]
            S_p_X = vehicle_coords[i][6] - 320
            S_p_Y = vehicle_coords[i][7] -188
            E_point = vehicle_coords[i][8]
            E_p_X = vehicle_coords[i][9] - 320
            E_p_Y = vehicle_coords[i][10] -188

            y1, x1, z1 = yxz(min_point, min_p_X, min_p_Y)
            y2, x2, z2 = yxz(S_point, S_p_X, S_p_Y)
            y3, x3, z3 = yxz(E_point, E_p_X, E_p_Y)

            y = (y2 + y3)/2
            x = (x2 + x3)/2
            scale = (((y2-y3)**2 + (x2-x3)**2)**(1/2) )/2
            # print(scale)

            marker.id = 100 + i

            marker.color.r = 0.0 + i/10
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0

            marker.scale.x = scale/500
            marker.scale.y = scale/500
            marker.scale.z = 1

            marker.pose.position.x = x/500
            marker.pose.position.y = y/500
            marker.pose.position.z = 0.4

            markerArray.markers.append(marker)

        if ( len(person_coords) < old_personCount):
            for i in range(old_personCount - len(person_coords)):
                marker.scale.x = 0
                marker.scale.y = 0
                marker.scale.z = 0
                marker.id = 100 + i + len(person_coords)
                markerArray.markers.append(marker)

        old_personCount = len(person_coords)

    except:
        print("marker update error")




if __name__ == '__main__':
    rospy.init_node('foxglove_object_node')
    rospy.loginfo("foxglove object monitor node start")

    rate = rospy.Rate(30)

    sub_coords = rospy.Subscriber("/objects/objects_coords", Float64MultiArray, callback=coord_read)
    pub_object = rospy.Publisher("/foxglove_node/objects", MarkerArray, queue_size=2)

    marker_init()

    while not rospy.is_shutdown():

        if coord_resived :
            pointUpdate()
            coord_resived = False

        pub_object.publish(markerArray)

        rate.sleep()
