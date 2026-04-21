#!/usr/bin/python
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import rospy
import math

rospy.init_node('clock')
pub = rospy.Publisher('/points', PointCloud, queue_size=10)
r = rospy.Rate(1)

while not rospy.is_shutdown():
    secs = int(rospy.Time.now().to_sec() % 60)
    angle = -secs / 60.0 * 2 * math.pi
    pc = PointCloud()
    pc.header.stamp = rospy.Time.now()
    pc.header.frame_id = 'base_link'
    pc.points.append( Point32(math.cos(angle), math.sin(angle), 0))
    pub.publish(pc)
    r.sleep()
