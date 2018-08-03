#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from time import sleep
import math
import matplotlib.pyplot as plt
import numpy

class PointsDisplay:

    def __init__(self):
        """Initialize.
        """
        rospy.Subscriber('/lighthouse_fit_display', Float32MultiArray, self.callback_display)

    def display_test(self):
 
        plt.ion()
        plt.figure(1)
        #plt.axis([-200,1500,-200,1500])
        plt.axis([-10,60,-10,60])

        # Plot raw data.
        x_axis = []
        y_axis = []
        for i in range(50):
            x_axis.append(i)
            y_axis.append(i)
        plt.hold(True)
        plt.plot(x_axis, y_axis, 'r.')

        #plt.plot(0,0, 'b.')

        #plt.show()
        plt.draw()
        #sleep(1)
        #plt.close(1)

    def callback_display(self,data):
 
        plt.ion()
        #plt.figure(1)
        plt.figure(figsize=(15, 15)) 
        plt.axis([-2.0,2.0,-2.0,2.0])

        # Plot raw data.
        x_axis = []
        y_axis = []
        #print(len(data.data))
        len_data = len(data.data)
        for i in range(0,len_data-5,2):
            x_axis.append(data.data[i])
            y_axis.append(data.data[i+1])
        #plt.hold(True)
        plt.plot(x_axis, y_axis, 'b.')

        x1 = numpy.linspace(-2.0, 2.0, 5) 
        line1_a = data.data[len_data-4]
        line1_b = data.data[len_data-3]
        y1 = line1_a * x1 + line1_b
        plt.plot(x1, y1, color="red") 

        if False:
            line2_a = data.data[len_data-2]
            line2_b = data.data[len_data-1]
            y2 = line2_a * x1 + line2_b
            plt.plot(x1, y2, color="green") 

        #plt.plot(data.data[i],data.data[i], 'r.')

        #plt.show()
        plt.draw()
        sleep(1.5)
        plt.close()
        #plt.clf()

def main():
    points_display = PointsDisplay()
    rospy.init_node('triangle_points_display', anonymous=True)

    #points_display.display_test()
    rospy.spin()

if __name__ == "__main__":
    main()
