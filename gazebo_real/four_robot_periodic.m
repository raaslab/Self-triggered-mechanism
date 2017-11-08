function fourrobot_self_movtar_cenEKF(~,~)


global N omega_max tarx_hat tary_hat i_store...
    iself_count self_tolerance gVimid id Sigma_hat timer_cnt ave_com ave_err T

com_time=zeros(1,N);
errp=zeros(1,N);
abserrp=zeros(1,N);

robot_0_pub_vel = rospublisher('/robot_0/cmd_vel');
robot_1_pub_vel = rospublisher('/robot_1/cmd_vel');
robot_2_pub_vel = rospublisher('/robot_2/cmd_vel');
robot_3_pub_vel = rospublisher('/robot_3/cmd_vel');

robot_pub_vel=[robot_0_pub_vel;robot_1_pub_vel;robot_2_pub_vel;robot_3_pub_vel];
robot_pub_msg = rosmessage('geometry_msgs/Twist');
% publish target vel_cmd
target_pub_vel = rospublisher('/robot_4/cmd_vel');
target_pub_msg = rosmessage('geometry_msgs/Twist');


%subscribe the pose topic to get the positon if the robot
posesub_0 = rossubscriber('/robot_0/pose');
robot0_pose = receive(posesub_0,3);
posesub_1 = rossubscriber('/robot_1/pose');
robot1_pose = receive(posesub_1,3);
posesub_2 = rossubscriber('/robot_2/pose');
robot2_pose = receive(posesub_2,3);
posesub_3 = rossubscriber('/robot_3/pose');
robot3_pose = receive(posesub_3,3);
%subscribe target pose
% posesub_tar = rossubscriber('/robot_3/pose');
% target_pose = receive(posesub_tar,3);

%transform the quaternion to the roll, pitch, yaw%[robot0_roll, robot0_pitch, robot0_yaw]
[~, ~, robot0_yaw] = quattoangle([robot0_pose.Orientation.W robot0_pose.Orientation.X robot0_pose.Orientation.Y robot0_pose.Orientation.Z]);
[~, ~, robot1_yaw] = quattoangle([robot1_pose.Orientation.W robot1_pose.Orientation.X robot1_pose.Orientation.Y robot1_pose.Orientation.Z]);
[~, ~, robot2_yaw] = quattoangle([robot2_pose.Orientation.W robot2_pose.Orientation.X robot2_pose.Orientation.Y robot2_pose.Orientation.Z]);
[~, ~, robot3_yaw] = quattoangle([robot3_pose.Orientation.W robot3_pose.Orientation.X robot3_pose.Orientation.Y robot3_pose.Orientation.Z]);

%collect the planar position and orientation (yaw) of the robot
robot0_pose_last=[robot0_pose.Position.X, robot0_pose.Position.Y, robot0_yaw];
robot1_pose_last=[robot1_pose.Position.X, robot1_pose.Position.Y, robot1_yaw];
robot2_pose_last=[robot2_pose.Position.X, robot2_pose.Position.Y, robot2_yaw];
robot3_pose_last=[robot3_pose.Position.X, robot3_pose.Position.Y, robot3_yaw];

robot_pose_last=[robot0_pose_last;robot1_pose_last;robot2_pose_last;robot3_pose_last];
robot_pose_px=robot_pose_last(:,1);
robot_pose_py=robot_pose_last(:,2);
%transfrom the positon to the angular
robot0_theta=positiontoangularfun((robot0_pose_last(2)-tary_hat),(robot0_pose_last(1)-tarx_hat));
robot1_theta=positiontoangularfun((robot1_pose_last(2)-tary_hat),(robot1_pose_last(1)-tarx_hat));
robot2_theta=positiontoangularfun((robot2_pose_last(2)-tary_hat),(robot2_pose_last(1)-tarx_hat));
robot3_theta=positiontoangularfun((robot3_pose_last(2)-tary_hat),(robot3_pose_last(1)-tarx_hat));

robot_itheta=[robot0_theta;robot1_theta;robot2_theta;robot3_theta];
robot_itheta=orderlythetafun(robot_itheta);
[robot_itheta(4)-robot_itheta(3),robot_itheta(3)-robot_itheta(2),robot_itheta(2)-robot_itheta(1)]


%collect the data in compact form
robot_theta=[(robot_itheta(4)-2*pi);robot_itheta;(robot_itheta(1)+2*pi)]; %virtual agent 0th:=agent N-2pi;virtual agent 7th:=agent 1st+ 2pi

%define the id of all  the robots
for i=1:N
     id(i)=i;
end

%%%transform the centralized EKF to self-triggered
%  [tarx_hat, tary_hat, Sigma_hat, tarxtrue,tarytrue] = KF_pioneer (...
%  tarx_hat, tary_hat, Sigma_hat, robot_pose_px, robot_pose_py,id, timer_cnt);
 
        
        %calculate the dirsed angular orientation next step
        for i=1:N
                       
                       ubdi=omega_max*T*iself_count(i)/2; %upper bound
                       r=omega_max*T*iself_count(i); % the prediciton range of neighbors' motion
                       gVimid(i)=1/4*(i_store(i,2)+2*robot_itheta(i)+i_store(i,1)); % the midepoint of i's guaranteed Voronoi set 
                  
                       errp(i)=gVimid(i)-robot_itheta(i); 
                       abserrp(i)=abs(gVimid(i)-robot_itheta(i));
                       proximity=max(abserrp(i),self_tolerance); % the degree agent goes towards midpoint of its guaranteed Voronoi set
                       
                       if ((ubdi==0)||(ubdi>=proximity)||(i_store(i,2)-r<=robot_itheta(i))||(i_store(i,1)+r>=robot_itheta(i)))
                             %if (ubdi>=proximity)  
                              com_time(i)=1;
                             
                             i_store(i,:)=[robot_theta(i),robot_theta(i+2)];% update the stored memory of its neighbors
                             iself_count(i)=1; % reset the number of omega*T

                             %itheta(i,k+1)=itheta(i,k)+1/4*T*(theta(i+2,k)-2*theta(i+1,k)+theta(i,k));
                            utemp_angle=i_store(i,2)-2*robot_theta(i+1)+i_store(i,1);

                            u_angle_abs=min(omega_max,abs(1/4*utemp_angle));
                            u_angle_sign=sign(utemp_angle);
                            [linear_vel, angle_vel]=edge_vd_cmd(robot_pose_last,i,u_angle_abs,u_angle_sign);
            
                         else
              
                             %R(2*i-1:2*i,k+1)=R(2*i-1:2*i,k); %keep memory
                             com_time(i)=0;
                             
                             
                              iself_count(i)= iself_count(i)+1;
             
                             if (abserrp(i)>=ubdi+omega_max)
                             u_angle_abs=abs(omega_max);
                             elseif (abserrp(i)<=ubdi)
                             u_angle_abs=0;
                             else
                             u_angle_abs=abs(abserrp(i)-ubdi); 
                             end
                             u_angle_sign=sign(errp(i));
                             [linear_vel, angle_vel]=edge_vd_cmd(robot_pose_last,i,u_angle_abs,u_angle_sign);
                 
                        end                                                             
                         robot_pub_msg.Linear.X=linear_vel;
                         robot_pub_msg.Angular.Z=angle_vel;
                         send(robot_pub_vel(i,:),robot_pub_msg);
        end
                         %publish target vel_cmd
                         target_pub_msg.Linear.X=0.00;
                         target_pub_msg.Angular.Z=0.00;
                         %target_pub_msg.Angular.Z=0.05;
                         send(target_pub_vel,target_pub_msg);
                         
                         
                         ave_err(timer_cnt)=sum(abserrp)/N;
                         figure(1);
                         title('average error of uniform distribution')
                         cla; hold on;
                         plot(ave_err(1:timer_cnt), 'ro','MarkerFaceColor','r')
                         
                         ave_com(timer_cnt)=sum(com_time)/N;
                         figure(2);
                         title('average communication among robots')
                         cla; hold on;
                         plot(ave_com(1:timer_cnt), 'ro','MarkerFaceColor','r')
                         
                         
                            timer_cnt=timer_cnt+1; 
                                                 
                         %plot the boundary, robot, target
                         figure(3);
                         cla; hold on;axis equal
                            p1x=[2.68 -3.7];
                            p1y=[10.9 1.39];
                            line(p1x,p1y), hold on

                            p2x=[-3.7 -2.56];
                            p2y=[1.39 -8.5];
                            line(p2x,p2y), hold on

                            p3x=[-2.56 7.38];
                            p3y=[-8.5 -0.688];
                            line(p3x,p3y), hold on

                            p4x=[7.38 2.68];
                            p4y=[-0.688 10.9];
                            line(p4x,p4y), hold on
                         
                         for i = 1 : size(robot_itheta,1)
                         %figure(2);
                         %h(1) = covarianceEllipse([x_hat;y_hat],Sigma_hat,[1 0 0],11.82);
                         %h(2) = plot(x_hat,y_hat,'rs','MarkerSize',8);
                         %h(3) = plot(x_true,y_true,'bp','MarkerSize',8);
                           h(1)=plot(robot_pose_px(i,1), robot_pose_py(i,1),'ro','MarkerFaceColor','r');
                           h(2) = covarianceEllipse([tarx_hat;tary_hat],Sigma_hat,[1 0 0],11.82);
                           h(3)=plot(tarx_hat,tary_hat,'rs','MarkerSize',8);
                           h(4)=plot(tarxtrue,tarytrue,'bp','MarkerSize',8);
                           h(5)=plot ([robot_pose_px(i,1) tarx_hat], [robot_pose_py(i,1) tary_hat],':');
                         end
                         
end    

