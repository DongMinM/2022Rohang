import rospy
from std_msgs.msg import String

rospy.init_node('trigger')
pub = rospy.Publisher('code',String)

while True:
    trg = input()
    if trg == '1':
        pub.publish(trg)
    elif trg == '2':
        pub.publish(trg)
    else:
        print('Check again (only 1 and 2)')