#!/usr/bin/env python
import rospy
from opencv_apps.msg import CircleArrayStamped
from std_msgs.msg import Float32MultiArray

pub = None

def callback(data):
    x_array = []
    y_array = []
    for circle in data.circles:
        x_array.append(circle.center.x)
        y_array.append(circle.center.y)

    float_array = Float32MultiArray()
    float_array.data += x_array
    float_array.data += y_array

    pub.publish(float_array)

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('converter', anonymous=True)

    global pub
    pub = rospy.Publisher('/hough_circles/circles_float_array', Float32MultiArray, queue_size=10)
    rospy.Subscriber("/hough_circles/circles", CircleArrayStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
