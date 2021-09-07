#! /usr/bin/env python
import rospy
import math
import qwiic_vl53l1x
import RPi.GPIO as GPIO
import signal
import atexit
from rospy.core import rospyinfo
from sensor_msgs.msg import LaserScan

if __name__ == "__main__":
    rospy.init_node("tof")
    pub = rospy.Publisher("/laser_scan",LaserScan,queue_size=1000)

    atexit.register(GPIO.cleanup)
    
    servopin=17
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(servopin,GPIO.OUT,initial=False)
    p=GPIO.PWM(servopin,50)	#50HZ:频率就是周期脉冲的周期的倒数
    p.start(0)					#start(initdutycycle)：占空比0-100间，0表示暂不输出
    rospy.sleep(2)

    msg=LaserScan()
    msg.header.frame_id = "map"
    msg.angle_min = 0                   # start angle of the scan [rad]
    msg.angle_max = math.pi            # end angle of the scan [rad]
    msg.angle_increment = math.pi/90  # angular distance between measurements [rad]
    msg.range_min = 0.02               # minimum range value [m]
    msg.range_max = 4                 # maximum range value [m]
    msg.ranges = []
    msg.intensities = []

    tof = qwiic_vl53l1x.QwiicVL53L1X()

    if (tof.sensor_init() == None):     # Begin returns 0 on a good init
        print("Sensor online!")					 
	    
    tof.set_distance_mode(2)
    tof.set_inter_measurement_in_ms(40)

    while not rospy.is_shutdown():
        try:
            tof.start_ranging()
            msg.ranges = []
            for i in range(90):

                p.ChangeDutyCycle(2.5+2*i/18)			#设置转动角度
                rospy.sleep(0.02)						#等该20ms周期结束  
                p.ChangeDutyCycle(0)					#归零信号  

                msg.ranges.append(tof.get_distance()/1000)    #convert to meter
                rospy.sleep(0.04)

            msg.header.stamp=rospy.Time.now()
            pub.publish(msg)

        except Exception:
            tof.stop_ranging() 

tof.stop_ranging()
GPIO.cleanup()