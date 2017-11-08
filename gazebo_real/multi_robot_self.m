function multi_robot_self(~,~)


global N omega_max tarx tary i_store iself_count self_tolerance gVimid ave_com timer_cnt T

com_time=zeros(1,N);


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
        for i=1:N
                       
                       ubdi=omega_max*T*iself_count(i)/2; %upper bound
                       r=omega_max*T*iself_count(i); % the prediciton range of neighbors' motion
                       gVimid(i)=1/4*(i_store(i,2)+2*robot_itheta_last(i)+i_store(i,1)); % the midepoint of i's guaranteed Voronoi set 
                  
                       errp=gVimid(i)-robot_itheta_last(i); 
                       abserrp=abs(gVimid(i)-robot_itheta_last(i));
                       proximity=max(abserrp,self_tolerance); % the degree agent goes towards midpoint of its guaranteed Voronoi set
                       
                       if ((ubdi==0)||(ubdi>=proximity)||(i_store(i,2)-r<=robot_itheta_last(i))||(i_store(i,1)+r>=robot_itheta_last(i)))
                             
                            com_time(i)=1;
                           %if (ubdi>=proximity)    
                             i_store(i,:)=[robot_theta_last(i),robot_theta_last(i+2)];% update the stored memory of its neighbors
                             iself_count(i)=1; % reset the number of omega*T

                             %itheta(i,k+1)=itheta(i,k)+1/4*T*(theta(i+2,k)-2*theta(i+1,k)+theta(i,k));
                            utemp_angle=i_store(i,2)-2*robot_theta_last(i+1)+i_store(i,1);

                            u_angle_abs=min(omega_max,abs(1/4*utemp_angle));
                            u_angle_sign=sign(utemp_angle);
                            [linear_vel, angle_vel]=edge_vd_cmd(robot_pose_last,i,u_angle_abs,u_angle_sign);
            
                       else
                            com_time(i)=0;
                             %R(2*i-1:2*i,k+1)=R(2*i-1:2*i,k); %keep memory
                              iself_count(i)= iself_count(i)+1;
             
                             if (abserrp>=ubdi+omega_max)
                             u_angle_abs=abs(omega_max);
                             elseif (abserrp<=ubdi)
                             u_angle_abs=0;
                             else
                             u_angle_abs=abs(abserrp-ubdi); 
                             end
                             u_angle_sign=sign(errp);
                             [linear_vel, angle_vel]=edge_vd_cmd(robot_pose_last,i,u_angle_abs,u_angle_sign);
                 
                        end                                                             
                         robot_pub_msg.Linear.X=linear_vel;
                         robot_pub_msg.Angular.Z=angle_vel;
                         send(robot_pub_vel(i,:),robot_pub_msg);
                         
        end
        
                         ave_com(timer_cnt)=sum(com_time)/N;
                         figure(2);
                         title('average communication among robots')
                         cla; hold on;
                         plot(ave_com(1:timer_cnt), 'ro','MarkerFaceColor','r')
                         
                         timer_cnt=timer_cnt+1; 
            
end    

