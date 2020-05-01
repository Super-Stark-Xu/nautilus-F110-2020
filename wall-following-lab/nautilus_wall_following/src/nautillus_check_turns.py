#!/usr/bin/env python

# checks whether the car made left / right turn
# Outputs the topic "/checkTurn"
# which is a String type that tells "none", "right", or "left"
# For example, if the car just made 90 degree turn to left side,
# this script will generate a String that says 'left' in "/checkTurn" topic.


import rospy
import math
from tf2_msgs.msg import TFMessage                      # listener
from tf.transformations import euler_from_quaternion    # for euler/qtnion conv
from std_msgs.msg import String                         # talker


# get the angle of moving direction from fixed coordinate
ANGL_FIXED_SPACE = 0.0
COUNT = 0
ANGL_INIT = 0.0
TURN_DIR = 'none'
def callback(msg):
    global ANGL_FIXED_SPACE
    global COUNT
    anglList = [msg.transforms[0].transform.rotation.x ,
                msg.transforms[0].transform.rotation.y ,
                msg.transforms[0].transform.rotation.z ,
                msg.transforms[0].transform.rotation.w ]
              # (note transforms[] always has length 1)
    (roll,pitch,yaw) = euler_from_quaternion(anglList)
    # print(roll * 180/math.pi , pitch * 180/math.pi , yaw * 180/math.pi, COUNT % 4)

    # filter garbage values from tf reading
    # : the tf reading shows pattern that (4n+0)th data is meaningful,
    #   while (4n+1), (4n+2), and (4n+3)th data are garbage values. 
    if  COUNT % 4 == 0 :
        ANGL_FIXED_SPACE = yaw
    COUNT = (COUNT + 1) % 400

    # determine if it made turn
    global ANGL_INIT
    global TURN_DIR
    if ANGL_FIXED_SPACE * ANGL_INIT >= 0:
        if ANGL_FIXED_SPACE - ANGL_INIT > 0.49 * math.pi:
            TURN_DIR = 'left'
            ANGL_INIT = ANGL_FIXED_SPACE
        elif ANGL_FIXED_SPACE - ANGL_INIT < -0.49 * math.pi:
            TURN_DIR = 'right'
            ANGL_INIT = ANGL_FIXED_SPACE
    elif ANGL_FIXED_SPACE < 0 and ANGL_INIT > 0:
        if (ANGL_FIXED_SPACE + 2*math.pi) - ANGL_INIT > 0.49 * math.pi:
            TURN_DIR = 'left'
            ANGL_INIT = ANGL_FIXED_SPACE
    elif ANGL_FIXED_SPACE > 0 and ANGL_INIT < 0:
        if ANGL_FIXED_SPACE - (ANGL_INIT + 2*math.pi) < -0.49 * math.pi:
            TURN_DIR = 'right'
            ANGL_INIT = ANGL_FIXED_SPACE




def listener():
    rospy.Subscriber("/tf", TFMessage, callback)




def talker():
    pub = rospy.Publisher('checkTurns', String, queue_size=1000)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # print("ANGL=",ANGL_FIXED_SPACE*180.0/math.pi, "INIT=", ANGL_INIT*180.0/math.pi , "turn=", TURN_DIR)
        pub.publish(TURN_DIR)
        rate.sleep()




if __name__ == '__main__':
    rospy.init_node('listner_talker', anonymous=True)
    listener()
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
