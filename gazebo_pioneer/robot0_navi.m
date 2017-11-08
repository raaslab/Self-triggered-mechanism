function robot0_navi(P,Q,R,S,robot0_pose_last,robot_0_pub_vel,robot_0_pub_msg)   
%function robot0_navi(boundary_id,P,Q,R,S,PQ,QR,RS,SP,robot0_pose_last,robot_0_pub_vel,robot_0_pub_msg)
%     % old code associated wtth boundary
%     if boundary_id==1 % move on the PQ
%         
%        if  robot0_pose_last(3)==PQ
%            robot0_move(robot0_pose_last,Q,QR,robot_0_pub_vel,robot_0_pub_msg);
%        else
%            robot0_rota(robot_0_pub_vel,robot_0_pub_msg);
%        end
%           
%            
%     elseif boundary_id==2 % move on the QR
%         
%         if robot0_pose_last(3)==QR
%            robot0_move(robot0_pose_last,R,RS,robot_0_pub_vel,robot_0_pub_msg);
%        else
%            robot0_rota(robot_0_pub_vel,robot_0_pub_msg);
%         end
%   
%         
%     elseif boundary_id==3 % move on the RS
%       
%         if robot0_pose_last(3)==RS
%            robot0_move(robot0_pose_last,S,SP,robot_0_pub_vel,robot_0_pub_msg);
%        else
%            robot0_rota(robot_0_pub_vel,robot_0_pub_msg);
%        end
%       
%         
%     elseif boundary_id==4 % move on the SP
%       
%         if robot0_pose_last(3)==SP
%            robot0_move(robot0_pose_last,P,PQ,robot_0_pub_vel,robot_0_pub_msg);
%        else
%            robot0_rota(robot_0_pub_vel,robot_0_pub_msg);
%        end
%         
%     else
%            robot0_stop(robot_0_pub_vel,robot_0_pub_msg);
%     
%     end

    % new code with vertices
  
	%cross point of the boundary
	%P=[-2.23;-8.13];
	%Q=[7.16;-0.60];
	%R=[2.73;9.96]; 
	%S=[-3.38;1.21];  
    %global vertex_flag;
    cornor_epsilon=1.5;
    linear_vel=0.1;
    angular_kp=0.05;%angular PID feedback parameter
    %vertex_flag=zeros(1); %indicate which vertex
    
    %robot is within the neigborhood of P	
    if sqrt((robot0_pose_last(1)-P(1,1))^2+(robot0_pose_last(2)-P(2,1))^2)<=cornor_epsilon || vertex_flag==2
    %it goes towards Q
    vertex_flag=2;
    robot_Q_angle=atan2pi((Q(2,1)-robot0_pose_last(2)),(Q(1,1)-robot0_pose_last(1)));%figure out the range of yaw---pi or 2pi
    robot_0_pub_msg.Linear.X=linear_vel;
    robot_0_pub_msg.Angular.Z=angular_kp*(robot_Q_angle-robot0_pose_last(3)); 
    send(robot_0_pub_vel,robot_0_pub_msg); 
         
    %robot is within the neigborhood of Q
    elseif sqrt((robot0_pose_last(1)-Q(1,1))^2+(robot0_pose_last(2)-Q(2,1))^2)<=cornor_epsilon || vertex_flag==3
    %it goes towards R
    vertex_flag=3;
    robot_R_angle=atan2pi((R(2,1)-robot0_pose_last(2)),(R(1,1)-robot0_pose_last(1)));%figure out the range of yaw---pi or 2pi
    robot_0_pub_msg.Linear.X=linear_vel;
    robot_0_pub_msg.Angular.Z=angular_kp*(robot_R_angle-robot0_pose_last(3)); 
    send(robot_0_pub_vel,robot_0_pub_msg); 
    
    %robot is within the neigborhood of R
    elseif sqrt((robot0_pose_last(1)-R(1,1))^2+(robot0_pose_last(2)-R(2,1))^2)<=cornor_epsilon || vertex_flag==4
    %it goes towards S
    vertex_flag=4;
    robot_S_angle=atan2pi((S(2,1)-robot0_pose_last(2)),(S(1,1)-robot0_pose_last(1)));%figure out the range of yaw---pi or 2pi
    robot_0_pub_msg.Linear.X=linear_vel;
    robot_0_pub_msg.Angular.Z=angular_kp*(robot_S_angle-robot0_pose_last(3)); 
    send(robot_0_pub_vel,robot_0_pub_msg); 

    %robot is within the neigborhood of S
    elseif sqrt((robot0_pose_last(1)-S(1,1))^2+(robot0_pose_last(2)-S(2,1))^2)<=cornor_epsilon || vertex_flag==1
    %it goes towards P
    vertex_flag=1;
    robot_P_angle=atan2pi((P(2,1)-robot0_pose_last(2)),(P(1,1)-robot0_pose_last(1)));%figure out the range of yaw---pi or 2 pi
    robot_0_pub_msg.Linear.X=linear_vel;
    robot_0_pub_msg.Angular.Z=angular_kp*(robot_P_angle-robot0_pose_last(3)); 
    send(robot_0_pub_vel,robot_0_pub_msg); 
    
    else
    % follow the previous command  
    
    end


end
