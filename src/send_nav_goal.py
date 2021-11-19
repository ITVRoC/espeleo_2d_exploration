import actionlib
import rospy
import espeleo_control.msg
from geometry_msgs.msg import Polygon, Point32
from nav_msgs.msg import Path


def callback(data):
    pub = rospy.Publisher("/polygon", Polygon, queue_size=1)
    try:
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.header)
        i = 0
        # define the action client and wait for the Action Server start
        action_client = actionlib.SimpleActionClient('espeleo_control_action', espeleo_control.msg.NavigatePathAction)
        action_client.wait_for_server(rospy.Duration.from_sec(3))
        
        espeleo_path = espeleo_control.msg.Path()
        espeleo_path.path = Polygon()  # need to fill this polygon with the 3D points of the path
        espeleo_path.header.frame_id = 'map'
        
        for pose in data.poses:
            print(pose,i)
            i=i+1
            point = Point32()
            point.x = pose.pose.position.x
            point.y = pose.pose.position.y
            point.z = 0
            espeleo_path.path.points.append(point)            

        pub.publish(espeleo_path.path)
        # send the goal and wait for it to finish
        goal = espeleo_control.msg.NavigatePathGoal()
        goal.path = espeleo_path
        action_client.send_goal(goal)
        
        # the robot can wait or even do other things while waiting for the navigation to finish
        rospy.loginfo("waiting for action result...")
        #action_client.wait_for_result()
        #action_result = action_client.get_result()
        # create a path
        #rospy.loginfo("action_result:%s", action_result)
    except Exception as e:
        rospy.logerr('Error sending action goal %s', e.message)

if __name__ == '__main__':
    rospy.init_node('goal_path_publisher', anonymous=True)
    rospy.Subscriber("/move_base_node/NavfnROS/plan", Path, callback)
    rospy.spin()