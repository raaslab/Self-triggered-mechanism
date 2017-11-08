function timerCallback(~,~)
    posesub_0 = rossubscriber('/robot_0/pose');
    robot0_pose = receive(posesub_0,3);
    [robot0_roll, robot0_pitch, robot0_yaw] = quat2angle([robot0_pose.Orientation.W robot0_pose.Orientation.X robot0_pose.Orientation.Y robot0_pose.Orientation.Z],'XYZ');
    %figure; hold on;
    %plot([robot0_pose.Position.X, robot0_pose.Position.Y],'.');
    %quiver(robot0_pose.Position.X, robot0_pose.Position.Y,cos(yaw),sin(yaw));
    %disp('hello')
    robot0_poseori=[robot0_pose.Position.X, robot0_pose.Position.Y,robot0_yaw];
    display(robot0_poseori)
end