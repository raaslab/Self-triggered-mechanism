 posesub_0 = rossubscriber('/robot_0/pose');
 %posesub_1 = rossubscriber('/robot_1/pose');
 
    robot0_pose = receive(posesub_0,3);
          [roll, pitch, yaw] = quat2angle([robot0_pose.Orientation.W robot0_pose.Orientation.X robot0_pose.Orientation.Y robot0_pose.Orientation.Z],'XYZ');
    figure; hold on;
    %plot([robot0_pose.Position.X, robot0_pose.Position.Y],'.');
   quiver(robot0_pose.Position.X, robot0_pose.Position.Y,cos(yaw),sin(yaw));
   tic
   toc
  while toc < 10
      toc
      robot0_pose = receive(posesub_0,3);
     % robot1_pose = receive(posesub_1,3);
      [roll, pitch, yaw] = quat2angle([robot0_pose.Orientation.W robot0_pose.Orientation.X robot0_pose.Orientation.Y robot0_pose.Orientation.Z],'XYZ');
      %plot([robot0_pose.Position.X, robot0_pose.Position.Y],'.');
      %plot(toc,yaw,'*');
      quiver(robot0_pose.Position.X, robot0_pose.Position.Y,cos(yaw),sin(yaw));
  end