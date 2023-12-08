import rospy
from geometry_msgs.msg import PoseStamped
import queue

def pose_callback(msg, msg_queue):
    msg_queue.put(msg)
    return

def filter():
    rospy.init_node("filter_node")
    msg_queue = queue.Queue(maxsize=1)
    publisher = rospy.Publisher("/mavros", PoseStamped, queue_size=1)
    subscriber = rospy.Subscriber("/vrpn_client_node/angy/pose", PoseStamped, lambda x: pose_callback(x, msg_queue))

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = msg_queue.get()
        publisher.publish(msg)
        rate.sleep()
    return


if __name__ == "__main__":
    
    filter()