#!/usr/bin/env python

import os
import sys
from pathlib import Path
import time
import rospy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Bool
LibraryPath = "/home/index1/index_ws/src/Melexis_Tactile_Sensor/UNIGE/Library"
sys.path.append(LibraryPath)
import TactaxisForce



# ########################################################################
# port = '/dev/ttyACM0' #enter the correct port number
# ######################################################################## 

class MelexisDriver():
    def __init__(self):
        self.sensor = None

        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, sharex=True, figsize=(12, 6))
        self.ax1.set_ylim(-0.1, 10)  
        self.ax2.set_ylim(-0.1, 6)

        self.normal_forces = []
        self.shear_forces = []
        self.shear_forces_h = []
        self.shear_forces_v = []
        self.time_data = []

        self.avg_empty_normal = 4.5474
        self.avg_empty_shear = 0.5140
        self.pulling_threshold = 0.12
        self.release_threshold = 0.15

        self.max_data_points = 60

        self.time2release = 5

        self.activation_sub = rospy.Subscriber("/melexis_activation", Bool, self.force_listener_callback)
        self.release_pub = rospy.Publisher("/melexis_release", Bool, queue_size=10)


    def force_listener_callback(self, _):
        rospy.loginfo("Force listener callback")
        self.normal_forces.clear()
        self.shear_forces.clear()
        self.time_data.clear()
        
        initial_time = time.time()

        if self.sensor is None:
            rospy.logerr("Sensor not connected")
            return

        while True:

            B,F = self.sensor.getForce(dual=True)


            self.normal_forces.append(-F[2])
            self.shear_forces.append(np.sqrt(np.power(F[0],2)+np.power(F[1],2)))
            # self.shear_forces_h.append(F[0])
            # self.shear_forces_v.append(F[1])
            self.time_data.append(time.time() - initial_time)

            if len(self.time_data) > 10:
                avg_normal = sum(self.normal_forces[-10:])/10
                avg_shear = sum(self.shear_forces[-10:])/10
                if len(self.time_data) >= self.max_data_points:
                    if self.shear_forces[-1] > (avg_shear + self.release_threshold):
                        time2release=-1
                        if time2release <= 0:
                            self.release_pub.publish(True)
                            return
            # self.update_plot(avg_normal, avg_shear)

            

    def update_plot(self, avg_normal, avg_shear):
        # Calculate the range of data to display
        start_index = max(0, len(self.time_data) - self.max_data_points)
        end_index = len(self.time_data)

        # Clear the plot
        self.ax1.clear()
        self.ax2.clear()
        
        # Plot the data
        self.ax1.plot(self.time_data[start_index:end_index], self.normal_forces[start_index:end_index], label='normal_forces')
        self.ax2.plot(self.time_data[start_index:end_index], self.shear_forces[start_index:end_index], label='shear_forces', color='orange')
        # ax2.plot(time_data[start_index:end_index], shear_forces_h[start_index:end_index], label='shear_forces', color='red')
        # ax2.plot(time_data[start_index:end_index], shear_forces_v[start_index:end_index], label='shear_forces', color='green')
        

        self.ax1.axhline(y=avg_normal, color='black', linestyle='--', label='avg_normal')
        self.ax1.fill_between(x=self.time_data[start_index:end_index],
                    y1 = avg_normal - self.pulling_threshold,
                    y2 = avg_normal + self.release_threshold,
                    alpha=0.25
                    )
        self.ax2.axhline(y=avg_shear, color='black', linestyle='--', label='avg shear')
        self.ax2.fill_between(x=self.time_data[start_index:end_index],
                    y1 = avg_shear - self.pulling_threshold,
                    y2 = avg_shear + self.release_threshold,
                    alpha=0.25
                    )

        self.ax1.legend()
        self.ax2.legend()
        
        self.ax1.set_ylim(-0.5, 10)  
        self.ax2.set_ylim(-0.5, 6)

        plt.draw()
        plt.pause(0.001)


    def connect2sensor(self, port):
        print("Enabling connection to the board")
        self.sensor = TactaxisForce.TactaxisForce(str(port),pathDataset="/home/index1/index_ws/src/Melexis_Tactile_Sensor/UNIGE/Data/Calib.csv")
        print("Connection established")

        
def main():
    global path
    global playback
    
    rospy.init_node('melexis_sensor', disable_signals=True)
    port = rospy.get_param('~port')
    print('Trying to open port:', port)
    melexis_driver = MelexisDriver()
    melexis_driver.connect2sensor(port)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


