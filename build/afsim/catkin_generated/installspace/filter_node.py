import rospy
from geometry_msgs.msg import PoseStamped
import queue
import threading

def pose_callback(msg, msg_queue, msg_lock):
    with msg_lock:
        if msg_queue.full():
            msg_queue.queue.clear()
        msg_queue.put(msg)
    return

def filter():
    print('Initialising filter_node')
    rospy.init_node("filter_node")
    msg_queue = queue.LifoQueue(maxsize=1)
    msg_lock = threading.Lock()

    publisher = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=1)
    subscriber = rospy.Subscriber("/vrpn_client_node/angy/pose", PoseStamped, lambda x: pose_callback(x, msg_queue, msg_lock))

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        
        try:
            with msg_lock:
                msg = msg_queue.get(block=False)
            publisher.publish(msg)
        except queue.Empty:
            print('Retrieving data from queue failed')
            continue    
        
        rate.sleep()
    return


if __name__ == "__main__":
    try:
        filter()
    except rospy.ROSInterruptException:
        pass