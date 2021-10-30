scale = 1.0

import sys

import copy

import rospy

import moveit_commander

import moveit_msgs.msg

import geometry_msgs.msg

#initialize moveit commander and rospy node

moveit_commander.roscpp_initialize(sys.argv)

rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

#instantiate robot commander object

robot = moveit_commander.RobotCommander()

#instantiate a PlanningSceneInterface object

scene = moveit_commander.PlanningSceneInterface()    

#instantiate a MoveGroupCommander object

move_group = moveit_commander.MoveGroupCommander("manipulator")



#plan a motion for this group to a desired pose for the end-effector

pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = 1.0
pose_goal.position.x = 0.4
pose_goal.position.y = 0.4
pose_goal.position.z = 0.4

move_group.set_pose_target(pose_goal)


#call the planner to compute the plan and execute it

plan = move_group.go(wait=True)

print("---INITIAL POSE GOAL---")
print(move_group.get_current_joint_values())


# Calling `stop()` ensures that there is no residual movement
move_group.stop()
# It is always good to clear your targets after planning with poses.
# Note: there is no equivalent function for clear_joint_value_targets()
move_group.clear_pose_targets()


waypoints = []

wpose = move_group.get_current_pose().pose

# WAYPOINTS TO TRACE UPPERCASE B    


wpose.position.z += scale * 0.2
waypoints.append(copy.deepcopy(wpose))

wpose.position.y += scale * 0.2
waypoints.append(copy.deepcopy(wpose))

wpose.position.z -= scale * 0.1
waypoints.append(copy.deepcopy(wpose))

wpose.position.y -= scale * 0.2
waypoints.append(copy.deepcopy(wpose))

wpose.position.y += scale * 0.2
waypoints.append(copy.deepcopy(wpose))

wpose.position.z -= scale * 0.1
waypoints.append(copy.deepcopy(wpose))

print("---POSE RIGHT BEFORE COMPLETING B---")
print(move_group.get_current_joint_values())

waypoints = []

wpose.position.y -= scale * 0.2
waypoints.append(copy.deepcopy(wpose))




# WAYPOINTS TO TRACE FIRST UPPERCASE L

wpose.position.z += scale * 0.2  # First move up (z)
waypoints.append(copy.deepcopy(wpose)) #add position to list of waypoints

print("---POSE AT THE TOP OF L---")
print(move_group.get_current_joint_values())

waypoints = []

wpose.position.z -= scale * 0.2  # Second move down in (z)
waypoints.append(copy.deepcopy(wpose)) #add position to list of waypoints

wpose.position.y += scale * 0.2  # Third move sideways (y)
waypoints.append(copy.deepcopy(wpose)) #add position to list of waypoints

wpose.position.y -= scale * 0.2
waypoints.append(copy.deepcopy(wpose))

# WAYPOINTS TO TRACE SECOND UPPERCASE L

wpose.position.z += scale * 0.2  # First move up (z)
waypoints.append(copy.deepcopy(wpose)) #add position to list of waypoints

wpose.position.z -= scale * 0.2  # Second move down in (z)
waypoints.append(copy.deepcopy(wpose)) #add position to list of waypoints

wpose.position.y += scale * 0.2  # Third move sideways (y)
waypoints.append(copy.deepcopy(wpose)) #add position to list of waypoints

print("---POSE AT THE RIGHT MOST OF L---")
print(move_group.get_current_joint_values())

waypoints = []

wpose.position.y -= scale * 0.2
waypoints.append(copy.deepcopy(wpose))



# We want the Cartesian path to be interpolated at a resolution of 1 cm
# which is why we will specify 0.01 as the eef_step in Cartesian
# translation.  We will disable the jump threshold by setting it to 0.0,
# ignoring the check for infeasible jumps in joint space, which is sufficient
# for this tutorial.
(plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
            )  # jump_threshold

move_group.execute(plan, wait=True)

moveit_commander.roscpp_shutdown()
