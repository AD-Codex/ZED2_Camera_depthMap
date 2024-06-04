#!/usr/bin/env python3

# # global network foxglove
# # roslaunch rosbridge_server rosbridge_websocket.launch

# # local network foxglove
# # . ~/.bashrc
# # ROS_LAN
# # roscore on master (jetson)

# alias ROS_LAN='MASTER_IP=192.168.8.103
# SLAVE_IP=192.168.8.103
# export ROS_MASTER_URI=http://$MASTER_IP:11311
# export ROS_HOSTNAME=$MASTER_IP
# export ROS_IP=$MASTER_IP
# echo ROS_LAN master config ready!'



import rospy
from sensor_msgs.msg import Imu
from tf.transformations import *
from visualization_msgs.msg import Marker
import numpy as np

linear_acceleration = [0.0, 0.0, 0.0, 0.0]
angular_velocity = [0.0, 0.0, 0.0, 0.0]
orientation = [0.0, 0.0, 0.0, 0.0]

Robo_car = Marker()


def Imu_read(msg: Imu):
    global linear_acceleration
    global angular_velocity
    global orientation

    linear_acceleration.clear()
    angular_velocity.clear()
    orientation.clear()

    linear_acceleration.append(msg.linear_acceleration.x)
    linear_acceleration.append(msg.linear_acceleration.y)
    linear_acceleration.append(msg.linear_acceleration.z)
    angular_velocity.append(msg.angular_velocity.x)
    angular_velocity.append(msg.angular_velocity.y)
    angular_velocity.append(msg.angular_velocity.z)
    orientation.append(msg.orientation.x)
    orientation.append(msg.orientation.y)
    orientation.append(msg.orientation.z)
    orientation.append(msg.orientation.w)



def init_robo_car():
    global Robo_car

    Robo_car.header.frame_id = "base_link"
    # Robo_car.parent_id = "base_link"
    Robo_car.frame_locked = True
    Robo_car.header.stamp = rospy.Time.now()

    Robo_car.type = Marker.CUBE
    Robo_car.id = 0
    Robo_car.action = 0

    # Scale
    Robo_car.scale.x = 1.1
    Robo_car.scale.y = 1.7
    Robo_car.scale.z = 0.8

    # Color
    Robo_car.color.r = 1.0
    Robo_car.color.g = 0.0
    Robo_car.color.b = 0.0
    Robo_car.color.a = 1.0

    # Pose
    Robo_car.pose.position.x = 0
    Robo_car.pose.position.y = -1
    Robo_car.pose.position.z = 0.4

    # matrix shift rol, pich, yaw
    q_orig = quaternion_from_euler(0, 0, 0)
    q_rot = quaternion_from_euler(3.14159, 0, 0)
    q_new = quaternion_multiply(q_rot, q_orig)
    # print(q_new)

    Robo_car.pose.orientation.x = q_new[0]
    Robo_car.pose.orientation.y = q_new[1]
    Robo_car.pose.orientation.z = q_new[2]
    Robo_car.pose.orientation.w = q_new[3]



def Robo_car_update():
    global Robo_car
    global linear_acceleration
    global angular_velocity
    global orientation

    Robo_car.pose.orientation.x = orientation[0]
    Robo_car.pose.orientation.y = orientation[2]
    Robo_car.pose.orientation.z = - orientation[1]
    Robo_car.pose.orientation.w = orientation[3]



if __name__ == '__main__':
    rospy.init_node('foxglove_car_node')
    rospy.loginfo("foxglove car node start")

    rate = rospy.Rate(30)

    sub_imu = rospy.Subscriber("/zed_node/imu", Imu, callback=Imu_read)
    pub_car = rospy.Publisher("/foxglove_node/car", Marker, queue_size=2)

    init_robo_car()

    while not rospy.is_shutdown():
        Robo_car_update()

        pub_car.publish(Robo_car)

        rate.sleep()
