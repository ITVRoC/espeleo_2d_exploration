import actionlib
import rospy
import espeleo_control.msg
from geometry_msgs.msg import Polygon


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

try:
    # define the action client and wait for the Action Server start
    action_client = actionlib.SimpleActionClient('espeleo_control_action', espeleo_control.msg.NavigatePathAction)
    action_client.wait_for_server(rospy.Duration.from_sec(3))
    
    # create a path
    espeleo_path = espeleo_control.msg.Path()
    espeleo_path.path = Polygon()  # need to fill this polygon with the 3D points of the path

    # send the goal and wait for it to finish
    goal = espeleo_control.msg.NavigatePathGoal()
    goal.path = espeleo_path  
    action_client.send_goal(goal)
    
    # the robot can wait or even do other things while waiting for the navigation to finish
    rospy.loginfo("waiting for action result...")
    #action_client.wait_for_result()
    #action_result = action_client.get_result()
    rospy.loginfo("action_result:%s", action_result)
except Exception as e:
    rospy.logerr('Error sending action goal %s', e.message)

if __name__ == '__main__':
    rospy.init_node('goal_path_publisher', anonymous=True)
    rospy.Subscriber("/move_base_node/NavfnROS/plan")