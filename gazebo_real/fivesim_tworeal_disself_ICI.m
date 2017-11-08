function fivesim_tworeal_disself_ICI(~,~)


global N omega_max tarx_hat tary_hat tarxtrue tarytrue i_store iself_count self_tolerance...
    gVimid Sigma_hat timer_cnt T ave_err tar_err  ave_com com_time...
    robot_pub_vel robot_pub_msg tftree
%com_time=zeros(1,N);
id=zeros(1,N);
errp=zeros(1,N);
abserrp=zeros(1,N);
errp_real=zeros(1,N);
abserrp_real=zeros(1,N);
tar_erri=zeros(1,N);

for i=1:N
     id(i)=i;
end

%to enlarge label
idb=[N,id,1];


% waitForTransform(tftree, '/global_origin', '/robot_0/base_link');
robot0ToGlobal = getTransform(tftree, '/global_origin', '/robot_0/base_link'); 
robot0Translation=robot0ToGlobal.Transform.Translation;
robot0Rotation=robot0ToGlobal.Transform.Rotation;


% waitForTransform(tftree, '/global_origin', '/robot_1/base_link');
robot1ToGlobal = getTransform(tftree, '/global_origin', '/robot_1/base_link'); 
robot1Translation=robot1ToGlobal.Transform.Translation;
robot1Rotation=robot1ToGlobal.Transform.Rotation;

% waitForTransform(tftree, '/global_origin', '/robot_2/base_link');
robot2ToGlobal = getTransform(tftree, '/global_origin', '/robot_2/base_link'); 
robot2Translation=robot2ToGlobal.Transform.Translation; %[x,y,z]
robot2Rotation=robot2ToGlobal.Transform.Rotation; %[quaternion]

% waitForTransform(tftree, '/global_origin', '/robot_3/base_link');
robot3ToGlobal = getTransform(tftree, '/global_origin', '/robot_3/base_link'); 
robot3Translation=robot3ToGlobal.Transform.Translation;
robot3Rotation=robot3ToGlobal.Transform.Rotation;

% waitForTransform(tftree, '/global_origin', '/robot_4/base_link');
robot4ToGlobal = getTransform(tftree, '/global_origin', '/robot_4/base_link'); 
robot4Translation=robot4ToGlobal.Transform.Translation;
robot4Rotation=robot4ToGlobal.Transform.Rotation;

% waitForTransform(tftree, '/global_origin', '/robot_5/base_link');
robot5ToGlobal = getTransform(tftree, '/global_origin', '/robot_5/base_link'); 
robot5Translation=robot5ToGlobal.Transform.Translation;
robot5Rotation=robot5ToGlobal.Transform.Rotation;

%transform the quaternion to the roll, pitch, yaw%[robot0_roll, robot0_pitch, robot0_yaw]
[~, ~, robot0_yaw] = quattoangle([robot0Rotation.W robot0Rotation.X robot0Rotation.Y robot0Rotation.Z]);
[~, ~, robot1_yaw] = quattoangle([robot1Rotation.W robot1Rotation.X robot1Rotation.Y robot1Rotation.Z]);
[~, ~, robot2_yaw] = quattoangle([robot2Rotation.W robot2Rotation.X robot2Rotation.Y robot2Rotation.Z]);
[~, ~, robot3_yaw] = quattoangle([robot3Rotation.W robot3Rotation.X robot3Rotation.Y robot3Rotation.Z]);
[~, ~, robot4_yaw] = quattoangle([robot4Rotation.W robot4Rotation.X robot4Rotation.Y robot4Rotation.Z]);
[~, ~, robot5_yaw] = quattoangle([robot5Rotation.W robot5Rotation.X robot5Rotation.Y robot5Rotation.Z]);

%collect the planar position and orientation (yaw) of the robot
robot0_pose_last=[robot0Translation.X, robot0Translation.Y, robot0_yaw];
robot1_pose_last=[robot1Translation.X, robot1Translation.Y, robot1_yaw];
robot2_pose_last=[robot2Translation.X, robot2Translation.Y, robot2_yaw];
robot3_pose_last=[robot3Translation.X, robot3Translation.Y, robot3_yaw];
robot4_pose_last=[robot4Translation.X, robot4Translation.Y, robot4_yaw];
robot5_pose_last=[robot5Translation.X, robot5Translation.Y, robot5_yaw];

robot_pose_last=[robot0_pose_last;robot1_pose_last;robot2_pose_last;robot3_pose_last;robot4_pose_last;robot5_pose_last];
robot_pose_px=robot_pose_last(:,1);
%to enlarge the position x
%robot_pose_px_enl=[robot_pose_px(N,1);robot_pose_px;robot_pose_px(1,1)];


robot_pose_py=robot_pose_last(:,2);
%to enlarge the position y
%robot_pose_py_enl=[robot_pose_py(N,1);robot_pose_py;robot_pose_py(1,1)];

%transfrom the positon to the angular
robot0_theta=positiontoangularfun((robot0_pose_last(2)-tary_hat(1)),(robot0_pose_last(1)-tarx_hat(1)));
robot1_theta=positiontoangularfun((robot1_pose_last(2)-tary_hat(2)),(robot1_pose_last(1)-tarx_hat(2)));
robot2_theta=positiontoangularfun((robot2_pose_last(2)-tary_hat(3)),(robot2_pose_last(1)-tarx_hat(3)));
robot3_theta=positiontoangularfun((robot3_pose_last(2)-tary_hat(4)),(robot3_pose_last(1)-tarx_hat(4)));
robot4_theta=positiontoangularfun((robot4_pose_last(2)-tary_hat(5)),(robot4_pose_last(1)-tarx_hat(5)));
robot5_theta=positiontoangularfun((robot5_pose_last(2)-tary_hat(6)),(robot5_pose_last(1)-tarx_hat(6)));
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
%[robot_itheta_last(3)-robot_itheta_last(2),robot_itheta_last(2)-robot_itheta_last(1)]


%collect the data in compact form
robot_theta=[(robot_itheta(N)-2*pi);robot_itheta;(robot_itheta(1)+2*pi)]; %virtual agent 0th:=agent N-2pi;virtual agent 7th:=agent 1st+ 2pi 
robot_theta_real=[(robot_itheta_real(N)-2*pi);robot_itheta_real;(robot_itheta_real(1)+2*pi)]; 
%define the id of all  the robots

        
        %calculate the dirsed angular orientation next step
        for i=1:N
                       
                       ubdi=omega_max*T*iself_count(i)/2; %upper bound
                       r=omega_max*T*iself_count(i); % the prediciton range of neighbors' motion
                       gVimid(i)=1/4*(i_store(i,2)+2*robot_itheta(i)+i_store(i,1)); % the midepoint of i's guaranteed Voronoi set 
                  
                       
                       errp_real(i)=robot_theta_real(i+2)-2*robot_theta_real(i+1)+robot_theta_real(i);
                       abserrp_real(i)=abs(errp_real(i)/4);
                      
                       errp(i)=gVimid(i)-robot_itheta(i);                      
                       abserrp(i)=abs(gVimid(i)-robot_itheta(i));
                       proximity=max(abserrp(i),self_tolerance); % the degree agent goes towards midpoint of its guaranteed Voronoi set
                       
                       % initially, set condition ubdi==0 to make robot
                       % communicate with its neighbors
                       if ((ubdi==0)||(ubdi>=proximity)||(i_store(i,2)-r<=robot_itheta(i))||(i_store(i,1)+r>=robot_itheta(i)))
                           
                           com_time(i)=com_time(i)+1;
                             %if (ubdi>=proximity)    
                             i_store(i,:)=[robot_theta(i),robot_theta(i+2)];% update the stored memory of its neighbors
                             %pose_x_store((3*i-2):3*i,1)=robot_pose_px_enl(i:i+2,1);
                             %pose_y_store((3*i-2):3*i,1)=robot_pose_py_enl(i:i+2,1);
                             
                             iself_count(i)=1; % reset the number of omega*T

                            %itheta(i,k+1)=itheta(i,k)+1/4*T*(theta(i+2,k)-2*theta(i+1,k)+theta(i,k));
                            utemp_angle=i_store(i,2)-2*robot_theta(i+1)+i_store(i,1);

                            u_angle_abs=min(omega_max,abs(1/4*utemp_angle));
                            u_angle_sign=sign(utemp_angle);
                            [linear_vel, angle_vel]=edge_vd_cmd(robot_pose_last,i,u_angle_abs,u_angle_sign);
                            
%                             %update estimate of target if acquire new info from its neighbors 
%                             [tarx_hat(i), tary_hat(i), Sigma_hat((2*i-1):2*i,:)] = KF_pioneer (...
%                             tarx_hat(i), tary_hat(i), Sigma_hat((2*i-1):2*i,:), pose_x_store((3*i-2):3*i,1), pose_y_store((3*i-2):3*i,1),idb(i:(i+2)), timer_cnt);
%                             tar_erri(i)=sqrt((tarx_hat(i)-tarxtrue)^2+(tary_hat(i)-tarytrue)^2);

                            %update estimate of target if acquire new info from its neighbors 
                            [tarx_hat, tary_hat, Sigma_hat] = KF_ICI_pioneer (...
                            tarx_hat, tary_hat, Sigma_hat, robot_pose_px, robot_pose_py,idb(i:(i+2)), timer_cnt);
                            tar_erri(i)=sqrt((tarx_hat(i)-tarxtrue)^2+(tary_hat(i)-tarytrue)^2);
          
                       else
                             %com_time(i)=0;
                             %R(2*i-1:2*i,k+1)=R(2*i-1:2*i,k); %keep memory
%                           [tarx_hat(i), tary_hat(i), Sigma_hat((2*i-1):2*i,:)] = KF_pioneer (...
%                           tarx_hat(i), tary_hat(i), Sigma_hat((2*i-1):2*i,:), robot_pose_px(i), robot_pose_py(i),id(i), timer_cnt);
                            [tarx_hat, tary_hat, Sigma_hat] = KF_ICI_pioneer (...
                            tarx_hat, tary_hat, Sigma_hat, robot_pose_px, robot_pose_py,id(i), timer_cnt);
                             tar_erri(i)=sqrt((tarx_hat(i)-tarxtrue)^2+(tary_hat(i)-tarytrue)^2);
                             
                       
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
%                          %publish target vel_cmd
%                          target_pub_msg.Linear.X=1;
%                          target_pub_msg.Angular.Z=0.52;
%                          send(target_pub_vel,target_pub_msg);
                         tar_vel_cmd();
                         
                         ave_err(timer_cnt)=sum(abserrp_real);
                         figure(1);
                         title('average error of uniform distribution')
                         cla; hold on;
                         %plot(ave_err(1:timer_cnt), 'ro','MarkerFaceColor','r')
                         plot(ave_err(1:timer_cnt), 'r')
                         
                         ave_com(timer_cnt)=sum(com_time);
                         figure(2);
                         title('average communication among robots')
                         cla; hold on;
                         %plot(ave_com(1:timer_cnt), 'ro','MarkerFaceColor','r')
                         plot(ave_com(1:timer_cnt), 'r')
                                                
                         tar_err(timer_cnt)=mean(tar_erri);
                         figure(3);
                         title('sum tar err')
                         cla; hold on;
                         %plot(ave_com(1:timer_cnt), 'ro','MarkerFaceColor','r')
                         plot(tar_err(1:timer_cnt), 'b')
                         
                         timer_cnt=timer_cnt+1; 
                         
                         %plot the boundary, robot, target
                         figure(4);
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
                           h(2) = covarianceEllipse([tarx_hat(i);tary_hat(i)],Sigma_hat((2*i-1):2*i,:),[1 0 0],11.82);
                           h(3)=plot(tarx_hat(i),tary_hat(i),'rs','MarkerSize',8);
                           h(4)=plot(tarxtrue,tarytrue,'bp','MarkerSize',8);
                           h(5)=plot ([robot_pose_px(i,1) tarx_hat(i)], [robot_pose_py(i,1) tary_hat(i)],':');
                         end               
end    