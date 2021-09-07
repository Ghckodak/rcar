#! /usr/bin/env python
import rospy
import serial    #import serial module  #import custom msg
from geometry_msgs.msg import Twist
ser = serial.Serial('/dev/ttyUSB0', 115200,timeout=1)   #open named port at 9600,1s timeot


def arduino():
    rospy.init_node('arduino', anonymous=True)
    sub=rospy.Subscriber("/cmd_vel", Twist, readCmdvel)
    rospy.spin()


def readCmdvel(data):
     #initilize an empty string for send the command message
    x=data.linear.x
    z=data.angular.z
    cmd=1000*mask(x)+mask(z)
    ser.write(cmd)
    rospy.loginfo(cmd)

def mask(num):  
    if (num<-2):     #mask the number [-2,2] to [1,255]
        result=1
    elif (num>2):
        result=255
    else:
        result=128+int(127.5*num)
    return result


if __name__ == '__main__':
#     try:
    arduino()
    
#     except 
#try and exceptstructure are exception handler
#try:
#    ser.write(s); #writ a string to port
#    response = ser.readall();#read a string from port
#    print (response)
#except:
#  ser.close()
