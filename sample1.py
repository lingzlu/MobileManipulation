import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math


moveit_commander.roscpp_initialize(sys.argv)

rospy.init_node('move_group_python_interface_tutorial',
                anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("left_arm")
display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory)

group.clear_pose_targets()

bx=0.319027463078
by=0.696932535372
bz=0.38963384769
#pose B
pose_target=geometry_msgs.msg.Pose()
pose_target.position.x=bx
pose_target.position.y=by
pose_target.position.z=bz

pose_target.orientation.x=0.89646727127
pose_target.orientation.y= -0.441927661622
pose_target.orientation.z=-0.0323472356087
pose_target.orientation.w=0.000172577279596

#pose_target.position.z=bz+.05
group.set_pose_target(pose_target)
poseB=group.plan()
group.go(wait=True)


ee=.1
theta=0
zz=bz
sign=1

waypoints = []
wpose=geometry_msgs.msg.Pose()

wpose.position.x = 0.319027463078
wpose.position.y = 0.696932535372
wpose.position.z = 0.38963384769

wpose.orientation.x = 0.89646727127
wpose.orientation.y = -0.441927661622
wpose.orientation.z = -0.0323472356087
wpose.orientation.w = 0.000172577279596

waypoints.append(copy.deepcopy(wpose))

while theta<=90:
    byee=math.cos(math.radians(theta))
    bxee=math.sin(math.radians(theta))
    wpose.position.x=bx+ee*bxee
    wpose.position.y=by+ee*byee
    waypoints.append(copy.deepcopy(wpose))
    theta+=5
    print theta

(plan_circ,fraction)=group.compute_cartesian_path(waypoints,0.01,0.0)

group.go()
