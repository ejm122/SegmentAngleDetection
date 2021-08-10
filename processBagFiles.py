import rospy
import rosbag
import numpy as np
from std_msgs.msg import Int32


def statistics():
    allBags = [rosbag.Bag('still_on_table.bag'),
               rosbag.Bag('hold_in_air.bag'),
               rosbag.Bag('stillPhantom.bag')]
    bagName = ["Still on table ",
               "Hold in air ",
               "Still on phantom "]

    whichTopic = ''

    for i in range(0, 3):
        positions = []
        timeStamps = []
        if i < 2:
            whichTopic = '/vicon/patient1'
        else:
            whichTopic = '/vicon/patient2'

        for topic, msg, t in allBags[i].read_messages(topics=[whichTopic]):
            positions.append([msg.position.x,
                              msg.position.y,
                              msg.position.z])
            timeStamps.append(msg.header.stamp.to_sec())
        print(bagName[i], "average is: ", np.mean(positions, axis=0))
        print(bagName[i], "stdev is: ", np.std(positions, axis=0))

        allBags[i].close

    positions1 = []
    positions2 = []
    timeStamps1 = []
    timeStamps2 = []
    for topic, msg, t in rosbag.Bag('movePhantom.bag').read_messages(topics=['/vicon/patient2']):
        if (0.20 < msg.position.x and msg.position.x < 0.22):
            positions1.append([msg.position.x,
                               msg.position.y,
                               msg.position.z])
            timeStamps1.append(msg.header.stamp.to_sec())
        elif (0.33 < msg.position.x and msg.position.x < 0.341):
            positions2.append([msg.position.x,
                               msg.position.y,
                               msg.position.z])
            timeStamps2.append(msg.header.stamp.to_sec())
    print("Move 90 (first half) average is: ", np.mean(positions1, axis=0))
    print("Move 90 (first half) stdev is: ", np.std(positions1, axis=0))
    print("Move 90 (second half) average is: ", np.mean(positions2, axis=0))
    print("Move 90 (second half) stdev is: ", np.std(positions2, axis=0))

    rosbag.Bag('movePhantom.bag').close
    # print(bagName[i],"timestamp is: ", timeStamps)
    rospy.init_node('average', anonymous=True)
    rospy.Subscriber('/vicon/patient1', Int32)

    # rospy.spin()


if __name__ == '__main__':
    statistics()