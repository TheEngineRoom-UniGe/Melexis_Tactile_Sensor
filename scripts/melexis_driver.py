#!/usr/bin/env python3

import rospy
import os
import sys
from pathlib import Path
from melexis.msg import MelexisForce
LibraryPath = '/home/simone/Desktop/catkin_ws/src/melexis/UNIGE/Library'
sys.path.append(LibraryPath)
import TactaxisForce


# ########################################################################
# port = '/dev/ttyACM0' #enter the correct port number
# ######################################################################## 

def publisher(port):
    pub = rospy.Publisher('/melexis_force' + port[-1], MelexisForce)

    print("Enabling connection to the board")
    with TactaxisForce.TactaxisForce(str(port),pathDataset="/home/simone/Desktop/catkin_ws/src/melexis/UNIGE/Data/Calib.csv") as Sensor:
        print("Connection established")

        while True:
            B,F = Sensor.getForce(dual=True)
            # MeasString = "Magnetic field measured is: \nBx00 = {Bx00},\nBz00 = {Bz00},\nBx01 = {Bx01},\nBz01 = {Bz01},\nBx10 = {Bx10},\nBz10 = {Bz10},\nBx11 = {Bx11},\nBz11 = {Bz11}".format(Bx00 = B[0],Bz00 = B[1],Bx01 = B[2],Bz01 = B[3],Bx10 = B[4],Bz10 = B[5],Bx11 = B[6],Bz11 = B[7])
            # print(MeasString)
            # print("Temperature is :"+str(B[-2])+" and " + str(B[-1]))
            # print("Force measured is: \nFx = {Fx},\nFy = {Fy},\nFz = {Fz}".format(Fx= F[0],Fy=F[1],Fz=F[2]))

            
            msg = MelexisForce()
            msg.header.stamp = rospy.Time.now()
            msg.Bx00 = B[0]
            msg.Bz00 = B[1]
            msg.Bx01 = B[2]
            msg.Bz01 = B[3]
            msg.Bx10 = B[4]
            msg.Bz10 = B[5]
            msg.Bx11 = B[6]
            msg.Bz11 = B[7]
            msg.Temperature = B[-2]
            msg.Fx = F[0]
            msg.Fy = F[1]
            msg.Fz = F[2]
            print(msg)

            pub.publish(msg)
        # print("End")

def main():
    global path
    global playback
    
    # print ("main called")
    rospy.init_node('tf_recording', disable_signals=True)
    port = rospy.get_param('~port')
    print('Trying to open port:', port)
    # os.system('sudo chmod a+rw /dev/ttyACM0')
    # os.system('sudo chmod a+rw /dev/ttyACM1')
    publisher(port)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


