function one_robot_navigation(~,~)

    cornor_epsilon=1.5;
    linear_vel=0.2;
    angular_kp=0.052;%angular PID feedback parameter
    
    global count;
    global vertex;
    

% % find the orientation (-pi-pi) of each line, + is counterclockwise order, betterto 2pi
% 
% PQ=atan2d((Q(2,1)-P(2,1)),(Q(1,1)-P(1,1)));
% QP=atan2d((P(2,1)-Q(2,1)),(P(1,1)-Q(1,1)));
% 
% QR=atan2d((R(2,1)-Q(2,1)),(R(1,1)-Q(1,1)));
% RQ=atan2d((Q(2,1)-R(2,1)),(Q(1,1)-R(1,1)));
% 
% RS=atan2d((S(2,1)-R(2,1)),(S(1,1)-R(1,1)));
% SR=atan2d((R(2,1)-S(2,1)),(R(1,1)-S(1,1)));
% 
% SP=atan2d((P(2,1)-S(2,1)),(P(1,1)-S(1,1)));
% PS=atan2d((S(2,1)-P(2,1)),(S(1,1)-P(1,1)));
% 
% % find the turning angle at each cornor, need function to +
% 
% PQR=QR-PQ;
% QRS=RS-QR;
% RSP=SP-RS;
% SPQ=PQ-SP;
% 
% 
% %the position of the target
% tarx=1.25;
% tary=0.39;

robot_0_pub_vel = rospublisher('/robot_0/cmd_vel');
robot_0_pub_msg = rosmessage('geometry_msgs/Twist');
% move_cmd = [0.1,0.0];
% rota_cmd = [0.0,0.1];

 
%subscribe the pose topic to get the positon if the robot
posesub_0 = rossubscriber('/robot_0/pose');  
robot0_pose = receive(posesub_0,3);
 
%transform the quaternion to the roll, pitch, yaw
[robot0_roll, robot0_pitch, robot0_yaw] = quattoangle([robot0_pose.Orientation.W robot0_pose.Orientation.X robot0_pose.Orientation.Y robot0_pose.Orientation.Z]);

%robot0_yaw

%collect the planar position and orientation of the robot
robot0_pose_last=[robot0_pose.Position.X, robot0_pose.Position.Y, robot0_yaw];
 
%check if the orientation is on the line, then give the velocity command
%robot0_rota();
%boundary_id=check_boundary(robot0_pose.Position.X,robot0_pose.Position.Y,P,Q,R,S) %(x2-x1)*(y-y1)=(y2-y1)*(x-x1), here, we give the tolerance ebuclong
%if (check_boundary(robot0_pose.Position.X,robot0_pose.Position.Y,P,Q,R,S)==1)

%robot0_navi(boundary_id,P,Q,R,S,PQ,QR,RS,SP,robot0_pose_last,robot_0_pub_vel,robot_0_pub_msg);
%robot0_navi(P,Q,R,S,robot0_pose_last,robot_0_pub_vel,robot_0_pub_msg);
% 
    vd = vertex(count,1:2);
    vd
    %sqrt((robot0_pose_last(1)-vd(1))^2+(robot0_pose_last(2)-vd(2))^2)
    if sqrt((robot0_pose_last(1)-vd(1))^2+(robot0_pose_last(2)-vd(2))^2)<=cornor_epsilon
        
        robot_0_pub_msg.Linear.X=0;
        count=count+1;
        if count>length(vertex);
            count=1;
        end
        vd=vertex(count,1:2);
            
        %robot_0_pub_msg.Angular.Z=angular_kp*(robot_Q_angle-robot0_pose_last(3));
    else
        robot_0_pub_msg.Linear.X=linear_vel;      
    end
        robot_vd_angle=atan2pi((vd(2)-robot0_pose_last(2)),(vd(1)-robot0_pose_last(1)));
        angle_diff_temp=regulateangular(robot_vd_angle-robot0_pose_last(3));
    
    if angle_diff_temp>pi/2 || angle_diff_temp<-pi/2
        robot_0_pub_msg.Linear.X=0;
    end
    
        robot_0_pub_msg.Angular.Z=angular_kp*(angle_diff_temp);
        angular_kp*(angle_diff_temp)
        send(robot_0_pub_vel,robot_0_pub_msg);        
end