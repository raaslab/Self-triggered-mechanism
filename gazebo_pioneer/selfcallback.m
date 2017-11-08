function selfcallback(~,~)
    N=3;
    %the position of the target
    tarx=1.25;
    tary=0.39;
      
    omega_max=1; %maximum angular velocity for each robot
    robot_nextitheta=zeros(N,1);
    robot_nextposeori=zeros(N,3);

    %subscribe the pose topic to get the positon if the robot
    posesub_0 = rossubscriber('/robot_0/pose');
    posesub_1 = rossubscriber('/robot_1/pose');
    posesub_2 = rossubscriber('/robot_2/pose');
    
    robot0_pose = receive(posesub_0,3);
    robot1_pose = receive(posesub_1,3);
    robot2_pose = receive(posesub_2,3);
    
    %transform the quaternion to the roll, pitch, yaw
    [robot0_roll, robot0_pitch, robot0_yaw] = quat2angle([robot0_pose.Orientation.W robot0_pose.Orientation.X robot0_pose.Orientation.Y robot0_pose.Orientation.Z],'XYZ');
    [robot1_roll, robot1_pitch, robot1_yaw] = quat2angle([robot1_pose.Orientation.W robot1_pose.Orientation.X robot1_pose.Orientation.Y robot1_pose.Orientation.Z],'XYZ');
    [robot2_roll, robot2_pitch, robot2_yaw] = quat2angle([robot2_pose.Orientation.W robot2_pose.Orientation.X robot2_pose.Orientation.Y robot2_pose.Orientation.Z],'XYZ');
    
    %collect the planar position and orientation of the robot
    robot0_poseori=[robot0_pose.Position.X, robot0_pose.Position.Y,robot0_yaw];
    robot1_poseori=[robot1_pose.Position.X, robot1_pose.Position.Y,robot1_yaw];
    robot2_poseori=[robot2_pose.Position.X, robot2_pose.Position.Y,robot2_yaw];
    
    robot_poseori=[robot0_poseori;robot1_poseori;robot2_poseori];
        
    %transfrom the positon to the angular 
    robot0_theta=positiontoangularfun((robot0_poseori(2)-tary),(robot0_poseori(1)-tarx));
    robot1_theta=positiontoangularfun((robot1_poseori(2)-tary),(robot1_poseori(1)-tarx));
    robot2_theta=positiontoangularfun((robot2_poseori(2)-tary),(robot2_poseori(1)-tarx));
    
    robot_itheta=[robot0_theta;robot1_theta;robot2_theta];
    robot_itheta=orderlythetafun(robot_itheta);
    
    %collect the data in compact form
    robot_theta=[(itheta(3)-360);robot_itheta;(itheta(1)+360)]; %virtual agent 0th:=agent N-2pi;virtual agent 7th:=agent 1st+ 2pi
   
    %calculate the dirsed orientation next step
    for i=1:3
    utemp_angular=robot_theta(i+2)-2*robot_theta(i+1)+robot_theta(i);
    %sigu=sign(robot_theta(i+2)-2*robot_theta(i+1)+robot_theta(i));
    
    u_angular=min(omega_max,abs(1/4*utemp_angular));
    
    %the orientation based on target at next step
    robot_nextitheta(i)=itheta(i)+sign(utemp_angular)*u_angular; 
    
    
    % mapping the angualr at the next_step to the positon in the pologon
    [robot_nextposeori(i,1),robot_nextposeori(i,2)]=angulartopositionfun(robot_nextitheta(i),tarx,tary);
    
    
    
    end
end