import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class MoveGroupPythonIntefaceTutorial(object):
    def __init__(self):
        FLAG=1
        super(MoveGroupPythonIntefaceTutorial, self).__init__()


        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial",
                        anonymous=True)

        robot = moveit_commander.RobotCommander("/iiwa/robot_description")

        scene = moveit_commander.PlanningSceneInterface("/iiwa/")


        group_name = "manipulator"
        group = moveit_commander.MoveGroupCommander(group_name,"/iiwa/robot_description",ns="iiwa")

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

        planning_frame = group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        eef_link = group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)
       
        group_names = robot.get_group_names()
        print(robot.get_current_state())


        a = 5
        planning_frame = group.get_planning_frame()
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
       

    def handtobase(self):
        group=self.group
        wpose = group.get_current_pose().pose
        print("W-POSE-Important")
        print(wpose)
        return wpose

    def plan_cartesian_path(self,mat,mat2,scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
        group = self.group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through:
        ##
        waypoints = []

        wpose = group.get_current_pose().pose
        # Right Side Down Facing ok Table
        # wpose.position.x = mat[0][3] + 0.045 # First move up (z)
        # wpose.position.y = mat[1][3] + 0.07
        # wpose.position.z = mat[2][3] + 0.11 # and sideways (y)

        #Straight Table Facing
        wpose.position.x = mat[0][3] + 0.02 # First move up (z)
        wpose.position.y = mat[1][3]- 0.07
        wpose.position.z = mat[2][3] + 0.3 # and sideways (y)

        # # # Straight Glass Space
        # wpose.position.x = mat[0][3] + 0.045 # First move up (z)
        # wpose.position.y = mat[1][3] - 0.05
        # wpose.position.z = mat[2][3]   # and sideways (y)


        # wpose.orientation.w=mat2[0]
        # wpose.orientation.x=mat2[1]
        # wpose.orientation.y=mat2[2]
        # wpose.orientation.z=mat2[3]
        print("ffffffff")
        waypoints.append(copy.deepcopy(wpose))
        print(waypoints)


        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def plan2(self):
   # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
        group = self.group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^0.90988
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through:
        ##
        waypoints = []

        wpose = group.get_current_pose().pose
        wpose2=copy.deepcopy(wpose)
        wpose.position.z = wpose.position.z + 0.32 # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # wpose.orientation.x=mat2[0]
        # wpose.orientation.y=mat2[1]
        # wpose.orientation.z=mat2[2]
        # wpose.orientation.w=mat2[3]
        # print("f222222")
        #wpose = group.get_current_pose().pose

        wpose = group.get_current_pose().pose
        # wpose.position.x = -0.02197
        # wpose.position.y = -0.59141
        # wpose.position.z = 0.66089

        # wpose.orientation.x=mat2[0]
        # wpose.orientation.y=mat2[1]
        # wpose.orientation.z=mat2[2]
        # wpose.orientation.w=mat2[3]
        # print("ffffffff")
        # waypoints.append(copy.deepcopy(wpose))

        # print(waypoints)
        # wpose.position.z = wpose.position.z - 0.44 # and sideways (y)

        # wpose.orientation.x=mat2[0]
        # wpose.orientation.y=mat2[1]
        # wpose.orientation.z=mat2[2]
        # wpose.orientation.w=mat2[3]
        # print("f222222")

        # wpose = group.get_current_pose().pose
        # wpose.position.x = 0.58707
        # wpose.position.y = -0.05867
        # wpose.position.z = 0.41652

        # wpose.orientation.x=mat2[0]
        # wpose.orientation.y=mat2[1]
        # wpose.orientation.z=mat2[2]
        # wpose.orientation.w=mat2[3]0.90988
        # print("ffffffff")
        # waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disab

        wpose = group.get_current_pose().pose
        wpose2=copy.deepcopy(wpose)
        wpose.position.z = wpose.position.x  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))


        wpose = group.get_current_pose().pose
        wpose2=copy.deepcopy(wpose)
        wpose.position.z = wpose.position.z + 0.02 # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = group.compute_cartesian_path(
                                        waypoints,   # waypoints to follow
                                        0.01,        # eef_step
                                        0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

    def pour(self,angle):
        group = self.group
        joint_goal = group.get_current_joint_values()

        joint_goal[7] = angle

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_joints = self.group.get_current_joint_values()       




    def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##re
    #  'srcSz' is [1 x 35]

        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        group.execute(plan, wait=True)