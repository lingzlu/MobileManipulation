moveit_msgs::RobotTrajectory trajectory_msg;
group.setPlanningTime(10.0);

double fraction = group.computeCartesianPath(waypoints,
                                             0.01,  // eef_step
                                             0.0,   // jump_threshold
                                             trajectory_msg, false);
// The trajectory needs to be modified so it will include velocities as well.
// First to create a RobotTrajectory object
robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(), "arm_group");

// Second get a RobotTrajectory from trajectory
rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory_msg);

// Thrid create a IterativeParabolicTimeParameterization object
trajectory_processing::IterativeParabolicTimeParameterization iptp;

// Fourth compute computeTimeStamps
success = iptp.computeTimeStamps(rt);
ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

// Get RobotTrajectory_msg from RobotTrajectory
rt.getRobotTrajectoryMsg(trajectory_msg);

// Finally plan and execute the trajectory
plan.trajectory_ = trajectory_msg;
ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",fraction * 100.0);
sleep(5.0);
group.execute(plan);
