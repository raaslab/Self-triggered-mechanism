function sixrobot_periodic_cenEKF(~,~)


global N omega_max tarx_hat tary_hat dottarx_hat dottary_hat tarxtrue tarytrue i_store...
    iself_count self_tolerance gVimid  Sigma_hat timer_cnt ave_com ave_err tar_err std_err con_pi_err T com_time...
    robot_pub_vel robot_pub_msg target_pub_vel target_pub_msg...
      posesub_0 posesub_1 posesub_2 posesub_3 posesub_4 posesub_5
%com_time=zeros(1,N);
abserrp=zeros(1,N);
abserrp_real=zeros(1,N);
con_pi_erri=zeros(1,N);
id=zeros(1,N);


% %subscribe the pose topic to get the positon if the robot
 %posesub_0 = rossubscriber('/robot_0/pose');
 robot0_pose = receive(posesub_0,3);
 %posesub_1 = rossubscriber('/robot_1/pose');
 robot1_pose = receive(posesub_1,3);
 %posesub_2 = rossubscriber('/robot_2/pose');
 robot2_pose = receive(posesub_2,3);
 %posesub_3 = rossubscriber('/robot_3/pose');
 robot3_pose = receive(posesub_3,3);
 robot4_pose = receive(posesub_4,3);
 robot5_pose = receive(posesub_5,3);
 
 
 robot0_pose=robot0_pose.Pose.Pose;
robot1_pose=robot1_pose.Pose.Pose;
robot2_pose=robot2_pose.Pose.Pose;
robot3_pose=robot3_pose.Pose.Pose;
robot4_pose=robot4_pose.Pose.Pose;
robot5_pose=robot5_pose.Pose.Pose;

   
% %subscribe target pose
% % posesub_tar = rossubscriber('/robot_3/pose');
% % target_pose = receive(posesub_tar,3);

%transform the quaternion to the roll, pitch, yaw%[robot0_roll, robot0_pitch, robot0_yaw]
[~, ~, robot0_yaw] = quattoangle([robot0_pose.Orientation.W robot0_pose.Orientation.X robot0_pose.Orientation.Y robot0_pose.Orientation.Z]);
[~, ~, robot1_yaw] = quattoangle([robot1_pose.Orientation.W robot1_pose.Orientation.X robot1_pose.Orientation.Y robot1_pose.Orientation.Z]);
[~, ~, robot2_yaw] = quattoangle([robot2_pose.Orientation.W robot2_pose.Orientation.X robot2_pose.Orientation.Y robot2_pose.Orientation.Z]);
[~, ~, robot3_yaw] = quattoangle([robot3_pose.Orientation.W robot3_pose.Orientation.X robot3_pose.Orientation.Y robot3_pose.Orientation.Z]);
[~, ~, robot4_yaw] = quattoangle([robot4_pose.Orientation.W robot4_pose.Orientation.X robot4_pose.Orientation.Y robot4_pose.Orientation.Z]);
[~, ~, robot5_yaw] = quattoangle([robot5_pose.Orientation.W robot5_pose.Orientation.X robot5_pose.Orientation.Y robot5_pose.Orientation.Z]);

%collect the planar position and orientation (yaw) of the robot
robot0_pose_last=[robot0_pose.Position.X, robot0_pose.Position.Y, robot0_yaw];
robot1_pose_last=[robot1_pose.Position.X, robot1_pose.Position.Y, robot1_yaw];
robot2_pose_last=[robot2_pose.Position.X, robot2_pose.Position.Y, robot2_yaw];
robot3_pose_last=[robot3_pose.Position.X, robot3_pose.Position.Y, robot3_yaw];
robot4_pose_last=[robot4_pose.Position.X, robot4_pose.Position.Y, robot4_yaw];
robot5_pose_last=[robot5_pose.Position.X, robot5_pose.Position.Y, robot5_yaw];

robot_pose_last=[robot0_pose_last;robot1_pose_last;robot2_pose_last;robot3_pose_last;robot4_pose_last;robot5_pose_last];
robot_pose_px=robot_pose_last(:,1);
robot_pose_py=robot_pose_last(:,2);
%transfrom the positon to the angular
robot0_theta=positiontoangularfun((robot0_pose_last(2)-tary_hat),(robot0_pose_last(1)-tarx_hat));
robot1_theta=positiontoangularfun((robot1_pose_last(2)-tary_hat),(robot1_pose_last(1)-tarx_hat));
robot2_theta=positiontoangularfun((robot2_pose_last(2)-tary_hat),(robot2_pose_last(1)-tarx_hat));
robot3_theta=positiontoangularfun((robot3_pose_last(2)-tary_hat),(robot3_pose_last(1)-tarx_hat));
robot4_theta=positiontoangularfun((robot4_pose_last(2)-tary_hat),(robot4_pose_last(1)-tarx_hat));
robot5_theta=positiontoangularfun((robot5_pose_last(2)-tary_hat),(robot5_pose_last(1)-tarx_hat));

%calculate the angle wrt true target
robot0_theta_real=positiontoangularfun((robot0_pose_last(2)-tarytrue),(robot0_pose_last(1)-tarxtrue));
robot1_theta_real=positiontoangularfun((robot1_pose_last(2)-tarytrue),(robot1_pose_last(1)-tarxtrue));
robot2_theta_real=positiontoangularfun((robot2_pose_last(2)-tarytrue),(robot2_pose_last(1)-tarxtrue));
robot3_theta_real=positiontoangularfun((robot3_pose_last(2)-tarytrue),(robot3_pose_last(1)-tarxtrue));
robot4_theta_real=positiontoangularfun((robot4_pose_last(2)-tarytrue),(robot4_pose_last(1)-tarxtrue));
robot5_theta_real=positiontoangularfun((robot5_pose_last(2)-tarytrue),(robot5_pose_last(1)-tarxtrue));

robot_itheta=[robot0_theta;robot1_theta;robot2_theta;robot3_theta;robot4_theta;robot5_theta];
robot_itheta_real=[robot0_theta_real;robot1_theta_real;robot2_theta_real;robot3_theta_real;robot4_theta_real;robot5_theta_real];

robot_itheta=orderlythetafun(robot_itheta);
robot_itheta_real=orderlythetafun(robot_itheta_real);

%[robot_itheta(4)-robot_itheta(3),robot_itheta(3)-robot_itheta(2),robot_itheta(2)-robot_itheta(1)]


%collect the data in compact form
robot_theta=[(robot_itheta(N)-2*pi);robot_itheta;(robot_itheta(1)+2*pi)]; %virtual agent 0th:=agent N-2pi;virtual agent 7th:=agent 1st+ 2pi
robot_theta_real=[(robot_itheta_real(N)-2*pi);robot_itheta_real;(robot_itheta_real(1)+2*pi)]; 


 
%define the id of all  the robots
for i=1:N
     id(i)=i;
     con_pi_erri(i)=abs(robot_theta_real(i+1)-robot_theta_real(i)-2*pi/N);
end

%%transform the centralized EKF to self-triggered%KF_pioneer
 [tarx_hat, tary_hat, Sigma_hat] = KF_pioneer (...
 tarx_hat, tary_hat, Sigma_hat, robot_pose_px, robot_pose_py,id, timer_cnt);
 tar_err(timer_cnt)=sqrt((tarx_hat-tarxtrue)^2+(tary_hat-tarytrue)^2);
        
        %calculate the dirsed angular orientation next step
        for i=1:N
            
                        com_time(i)=com_time(i)+1;
                        
                         
                        utemp_angle=robot_theta(i+2)-2*robot_theta(i+1)+robot_theta(i);
                        abserrp(i)=abs(utemp_angle/4);
                        abserrp_real(i)=abs((robot_theta_real(i+2)-2*robot_theta_real(i+1)+robot_theta_real(i))/4);

                        u_angle_abs=min(omega_max,abs(1/4*utemp_angle));
                        u_angle_sign=sign(utemp_angle);

                        %the orientation based on target at next step
                        %robot_theta_next(i)=robot_itheta_last(i)+u_angle_sign(i)*u_angle_abs(i);

                        %i_edge_last(i)=i_edge_flag(i);
                        %vd(i,1:2) = edge(i_edge_flag(i),3:4);

                         [linear_vel, angle_vel]=edge_vd_cmd(robot_pose_last,i,u_angle_abs,u_angle_sign);
                 
                         robot_pub_msg.Linear.X=linear_vel;
                         robot_pub_msg.Angular.Z=angle_vel;
                         send(robot_pub_vel(i,:),robot_pub_msg);
        end
                         tar_vel_cmd();
                         
                        ave_err(timer_cnt)=sum(abserrp_real);
                        %ave_err(timer_cnt)=5*mean(abserrp_real);
                        %std_err(timer_cnt)=5*std(abserrp_real);
                        %con_pi_err(timer_cnt)=sum(con_pi_erri);
                         
                         
%                          figure(1);
%                          %title('Average error of uniform distribution at each timestep')
%                          cla; hold on;
%                          %plot(ave_err(1:timer_cnt), 'ro','MarkerFaceColor','r')
%                          plot(ave_err(1:timer_cnt), 'r')
%                          %errorbar(ave_err(1:timer_cnt),std_err(1:timer_cnt),'r');
%                          %plot(con_pi_err(1:timer_cnt), 'r')
%                          
%                          ave_com(timer_cnt)=sum(com_time);
%                          figure(2);
%                          title('average communication among robots at each timestep')
%                          cla; hold on;
%                          %plot(ave_com(1:timer_cnt), 'ro','MarkerFaceColor','r')
%                          plot(ave_com(1:timer_cnt), 'b')
%                          
%                          figure(3);
%                          title('estimate target error')
%                          cla; hold on;
%                          %plot(ave_com(1:timer_cnt), 'ro','MarkerFaceColor','r')
%                          plot(tar_err(1:timer_cnt), 'b')
%                          
%                          timer_cnt=timer_cnt+1; 
                                                 
                         %plot the boundary, robot, target
                         figure(1);
                         cla; hold on;box on;axis equal
                         title('Constant communication with centralized EKF','fontsize',12)
                         xlabel({'$$x$$'},'Interpreter','latex','fontsize',11)
                         ylabel({'$$y$$'},'Interpreter','latex','fontsize',11)
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