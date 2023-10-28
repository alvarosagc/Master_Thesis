#!/usr/bin/env python3
""""" This script provides an interface to the robot in the Gazebo simulator."""
import rospy
from sensor_msgs.msg import LaserScan
from GroovesDetection_v7 import GroovesDetection
from matplotlib import pyplot as plt
import threading
#import math 


quit_event = threading.Event()

def set_grooves_labels(grooves, grooves_labels):

    if len(grooves) > 0:

        if grooves.shape[1] == 1:

            grooves_labels[0].set_text(str(int(grooves[0][0])))
            grooves_labels[0].set_position((grooves[0][0], grooves[1][0]+0.05))
            grooves_labels[1].set_text('')
            
        else: 

            grooves_labels[0].set_text(str(int(grooves[0][0])))
            grooves_labels[0].set_position((grooves[0][0], grooves[1][0]+0.05))
            grooves_labels[1].set_text(str(int(grooves[0][1])))
            grooves_labels[1].set_position((grooves[0][1], grooves[1][1]+0.05))
    else:

        grooves_labels[0].set_text('')
        grooves_labels[1].set_text('')

def front_laser_callback(msg):

    global front_laser
    front_laser = list(msg.ranges[int(405 - N/2):int(405 + N/2)])
    front_laser.reverse()
    
def rear_laser_callback(msg):

    global rear_laser
    rear_laser = list(msg.ranges[int(405 - N/2):int(405 + N/2)])

def show_graph():

    global front_laser
    global rear_laser
    #quit_event.clear()
    
    while not quit_event.is_set():

        rospy.sleep(0.025)
       
        groove_detection.update_sensor_data(rear_laser, front_laser)

        if len(groove_detection.front_grooves) > 0:

            grooves_x = groove_detection.front_grooves[0]
            grooves_y = groove_detection.front_grooves[1]

        else: 

            grooves_x = []
            grooves_y = []

        line1.set_ydata(groove_detection.front_laser)
        #line1.set_ydata([groove_detection.front_laser[i]*math.cos(math.radians(270)/810 *(-i + 202.5))  for i in range(len(groove_detection.front_laser))])
        line2.set_xdata(groove_detection.front_fitted_curve_index)
        line2.set_ydata(groove_detection.front_fitted_curve)
        line3.set_xdata(grooves_x)
        line3.set_ydata(grooves_y)
        set_grooves_labels(groove_detection.front_grooves, front_grooves_labels)


        if len(groove_detection.rear_grooves) > 0:

            grooves_x = groove_detection.rear_grooves[0]
            grooves_y = groove_detection.rear_grooves[1]

        else: 

            grooves_x = []
            grooves_y = []

        line4.set_ydata(groove_detection.rear_laser)
        line5.set_xdata(groove_detection.rear_fitted_curve_index)
        line5.set_ydata(groove_detection.rear_fitted_curve)
        line6.set_xdata(grooves_x)
        line6.set_ydata(grooves_y)
        set_grooves_labels(groove_detection.rear_grooves, rear_grooves_labels)        

N = 405
font = {'family' : 'normal',
        'weight' : 'bold',
        'size'   : 12}

plt.rc('font', **font)
plt.ion()
fig, (ax, ax2) = plt.subplots(2,1)

ax.set_title('Front Sensor',y=1.0,pad=-14)
ax.set_xlim([0,N])
ax.set_ylim([0.5,2.5])

line1, = ax.plot([0]*N, 'b', label='Raw data')
line2, = ax.plot([0]*N, 'k--', label='Fitted data')
line3, = ax.plot([0]*N, 'gs', label='Grooves')
ax.legend()
front_grooves_labels = [ax.text(0, 0, ''),ax.text(0, 0, '')]

ax2.set_title('Rear Sensor', y=1.0, pad=-14)
ax2.set_xlim([0,N])
ax2.set_ylim([0.5,2.5])

line4, = ax2.plot([0]*N, 'b', label='Raw data')
line5, = ax2.plot([0]*N, 'k--',label='Fitted data')
line6, = ax2.plot([0]*N, 'gs', label='Grooves')
ax2.legend()
rear_grooves_labels = [ax2.text(0, 0, ''),ax2.text(0, 0, '')]

front_laser = N * [0]
rear_laser = N * [0]
groove_detection = GroovesDetection()

rospy.init_node('show_graph')
laser_topic = rospy.Subscriber("/rotrac_e2/laser_front/scan/", LaserScan, front_laser_callback)
laser_topic2 = rospy.Subscriber("/rotrac_e2/laser_rear/scan/", LaserScan, rear_laser_callback)

plot_thread = threading.Thread(target= show_graph)
plot_thread.start()

if __name__ == "__main__":

    while not rospy.is_shutdown():
      
        plt.show(block=True)
        quit_event.set()
        rospy.spin()
