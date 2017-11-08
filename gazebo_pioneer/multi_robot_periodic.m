function multi_robot_periodic(~,~)

%global count;

global omega_max tarx tary
% global vd;
%global i_edge_last;
% global edge;
% global i_edge_flag;
%global roboti_vd_angle;
%robot_theta_next=zeros(1,N);
%     u_angle_abs=zeros(1,N);
%     u_angle_sign=zeros(1,N);

robot_0_pub_vel = rospublisher('/robot_0/cmd_vel');
robot_1_pub_vel = rospublisher('/robot_1/cmd_vel');
robot_2_pub_vel = rospublisher('/robot_2/cmd_vel');

robot_pub_vel=[robot_0_pub_vel;robot_1_pub_vel;robot_2_pub_vel];


robot_pub_msg = rosmessage('geometry_msgs/Twist');


%subscribe the pose topic to get the positon if the robot
posesub_0 = rossubscriber('/robot_0/pose');
robot0_pose = receive(posesub_0,3);
posesub_1 = rossubscriber('/robot_1/pose');
robot1_pose = receive(posesub_1,3);
posesub_2 = rossubscriber('/robot_2/pose');
robot2_pose = receive(posesub_2,3);

%transform the quaternion to the roll, pitch, yaw%[robot0_roll, robot0_pitch, robot0_yaw]
[~, ~, robot0_yaw] = quattoangle([robot0_pose.Orientation.W robot0_pose.Orientation.X robot0_pose.Orientation.Y robot0_pose.Orientation.Z]);
[~, ~, robot1_yaw] = quattoangle([robot1_pose.Orientation.W robot1_pose.Orientation.X robot1_pose.Orientation.Y robot1_pose.Orientation.Z]);
[~, ~, robot2_yaw] = quattoangle([robot2_pose.Orientation.W robot2_pose.Orientation.X robot2_pose.Orientation.Y robot2_pose.Orientation.Z]);


%collect the planar position and orientation (yaw) of the robot
robot0_pose_last=[robot0_pose.Position.X, robot0_pose.Position.Y, robot0_yaw];
robot1_pose_last=[robot1_pose.Position.X, robot1_pose.Position.Y, robot1_yaw];
robot2_pose_last=[robot2_pose.Position.X, robot2_pose.Position.Y, robot2_yaw];

robot_pose_last=[robot0_pose_last;robot1_pose_last;robot2_pose_last];
robot_pose_last
%transfrom the positon to the angular
robot0_theta=positiontoangularfun((robot0_pose_last(2)-tary),(robot0_pose_last(1)-tarx));
robot1_theta=positiontoangularfun((robot1_pose_last(2)-tary),(robot1_pose_last(1)-tarx));
robot2_theta=positiontoangularfun((robot2_pose_last(2)-tary),(robot2_pose_last(1)-tarx));

robot_itheta_last=[robot0_theta;robot1_theta;robot2_theta];
robot_itheta_last=orderlythetafun(robot_itheta_last);

%collect the data in compact form
robot_theta_last=[(robot_itheta_last(3)-2*pi);robot_itheta_last;(robot_itheta_last(1)+2*pi)]; %virtual agent 0th:=agent N-2pi;virtual agent 7th:=agent 1st+ 2pi


%calculate the dirsed angular orientation next step
for i=1:3
    
    utemp_angle=robot_theta_last(i+2)-2*robot_theta_last(i+1)+robot_theta_last(i);
    
    u_angle_abs=min(omega_max,abs(1/4*utemp_angle));
    u_angle_sign=sign(utemp_angle);
    
    %the orientation based on target at next step
    %robot_theta_next(i)=robot_itheta_last(i)+u_angle_sign(i)*u_angle_abs(i);
    
    %i_edge_last(i)=i_edge_flag(i);
    %vd(i,1:2) = edge(i_edge_flag(i),3:4);
    
   [linear_vel, angle_vel]=edge_vd_cmd(robot_pose_last,i,u_angle_abs,u_angle_sign);
    
%     % if the robot is close to the left vertex of the edge
%     if sqrt((robot_pose_last(i,1)-edge(i_edge_flag(i),1))^2+(robot_pose_last(i,2)-edge(i_edge_flag(i),2))^2)<=cornor_epsilon
%         % enter the neighborhood, first stop
%         %robot_pub_msg.Linear.X=0;
%         % if the velocity -, the edge -1;
%         if u_angle_sign<0
%             %select the previous edge
%             i_edge_flag(i)=i_edge_flag(i)-1;
%             if i_edge_flag(i)<=0;
%                 i_edge_flag(i)=4;
%             end
%             % select the left vertex of the previous edge
%             vd(i,1:2)=edge(i_edge_flag(i),1:2);
%             roboti_vd_angle=atan2pi((vd(i,2)-robot_pose_last(i,2)),(vd(i,1)-robot_pose_last(i,1)));
%             roboti_yaw=robot_pose_last(i,3);
%             [linear_vel, angle_vel]=velcmd(roboti_vd_angle, roboti_yaw,u_angle_abs);
%         else
%             % stay the current edge,
%             % i_edge_flag(i)=i_edge_flag(i);
%             % select the right vertex of the current edge
%             vd(i,1:2)=edge(i_edge_flag(i),3:4);
%             roboti_vd_angle=atan2pi((vd(i,2)-robot_pose_last(i,2)),(vd(i,1)-robot_pose_last(i,1)));
%             roboti_yaw=robot_pose_last(i,3);
%             [linear_vel, angle_vel]=velcmd(roboti_vd_angle, roboti_yaw,u_angle_abs);
%         end
%         
%         
%         % if the robot is close to the right vetex of the edge
%     elseif sqrt((robot_pose_last(i,1)-edge(i_edge_flag(i),3))^2+(robot_pose_last(i,2)-edge(i_edge_flag(i),4))^2)<=cornor_epsilon
%         % enter the neighborhood, first stop
%         %robot_pub_msg.Linear.X=0;
%         % if the velocity +, the edge +1;
%         if u_angle_sign>0
%             %select the next edge
%             i_edge_flag(i)=i_edge_flag(i)+1;
%             if i_edge_flag(i)>length(vertex);
%                 i_edge_flag(i)=1;
%             end
%             % select the right vertex of the next edge
%             vd(i,1:2)=edge(i_edge_flag(i),3:4);
%             roboti_vd_angle=atan2pi((vd(i,2)-robot_pose_last(i,2)),(vd(i,1)-robot_pose_last(i,1)));
%             roboti_yaw=robot_pose_last(i,3);
%             [linear_vel, angle_vel]=velcmd(roboti_vd_angle, roboti_yaw,u_angle_abs);
%         else
%             % stay the current edge,
%             % i_edge_flag(i)=i_edge_flag(i);
%             % select the left vertex of the current edge
%             vd(i,1:2)=edge(i_edge_flag(i),1:2);
%             roboti_vd_angle=atan2pi((vd(i,2)-robot_pose_last(i,2)),(vd(i,1)-robot_pose_last(i,1)));
%             roboti_yaw=robot_pose_last(i,3);
%             [linear_vel, angle_vel]=velcmd(roboti_vd_angle, roboti_yaw,u_angle_abs);
%         end
%         
%         % esle the robot is between the left & right vertex of the edge
%     else
%         
%         if u_angle_sign<0
%             vd(i,1:2)=edge(i_edge_flag(i),1:2);
%             roboti_vd_angle=atan2pi((vd(i,2)-robot_pose_last(i,2)),(vd(i,1)-robot_pose_last(i,1)));
%             roboti_yaw=robot_pose_last(i,3);
%             [linear_vel, angle_vel]=velcmd(roboti_vd_angle, roboti_yaw,u_angle_abs);
%         else
%             vd(i,1:2)=edge(i_edge_flag(i),3:4);
%             roboti_vd_angle=atan2pi((vd(i,2)-robot_pose_last(i,2)),(vd(i,1)-robot_pose_last(i,1)));
%             roboti_yaw=robot_pose_last(i,3);
%             [linear_vel, angle_vel]=velcmd(roboti_vd_angle, roboti_yaw,u_angle_abs);
%         end
%         
% end
     robot_pub_msg.Linear.X=linear_vel;
     robot_pub_msg.Angular.Z=angle_vel;
     send(robot_pub_vel(i,:),robot_pub_msg);
    
end


