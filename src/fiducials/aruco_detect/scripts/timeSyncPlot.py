#!/usr/bin/env python

import rospy
import message_filters
from fiducial_msgs.msg import FiducialTransformArray
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from threading import Thread
import queue

# Queue to hold time differences
time_diff_queue = queue.Queue()

def callback(nuc2_msg, nuc3_msg):
    nuc2_time = nuc2_msg.header.stamp
    nuc3_time = nuc3_msg.header.stamp
    time_diff = abs(nuc2_time.to_sec() - nuc3_time.to_sec())
    rospy.loginfo("Time difference between messages: {} seconds".format(time_diff))
    # Put the time difference in the queue
    time_diff_queue.put(time_diff)

def plotter():
    fig, ax = plt.subplots()
    xs, ys = [], []

    def animate(i, xs, ys):
        if not time_diff_queue.empty():
            time_diff = time_diff_queue.get()
            xs.append(i)
            ys.append(time_diff)
            xs = xs[-60:]  # Limit x and y lists to 20 items
            ys = ys[-60:]

            ax.clear()
            ax.plot(xs, ys)
            plt.title('Live Time Difference Graph before Synchronization')
            plt.ylabel('Time Difference (seconds)')
            plt.xlabel('Samples')

    ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=1000)
    plt.show()

def main():
    rospy.init_node('synchronization_node', anonymous=True)

    nuc2_sub = message_filters.Subscriber('/nuc2/fiducial_transforms', FiducialTransformArray)
    nuc3_sub = message_filters.Subscriber('/nuc3/fiducial_transforms', FiducialTransformArray)

    ats = message_filters.ApproximateTimeSynchronizer([nuc2_sub, nuc3_sub], 10, 0.1)
    ats.registerCallback(callback)

    # Start the plotting thread
    plot_thread = Thread(target=plotter)
    plot_thread.start()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
