%cross point of the boundary
% P=[-2.23;-8.13];
% Q=[7.16;-0.60];
% R=[2.73;9.96]; 
% S=[-3.38;1.21];
  global N
  global T
  global vertex
  global edge
  global i_edge_flag
  global cornor_epsilon
  global omega_max
  global k_wtov % simple transform from angle velocity (wrt to target) to the boundary linear_vel
  global angle_kp %angular PID feedback parameter
  global tarx_hat tary_hat tarxtrue tarytrue
  global vd % the next vertex robot is going to reach;
  %global i_edge_last;
  %global roboti_vd_angle;
  %global comm_flag;
  global i_store
  global iself_count 
  global self_tolerance
  global gVimid
  global ave_err ave_com com_time con_pi_err

  
  global timer_cnt
  global Sigma_hat
%   global id % update the EKF filter
%   global idb
 
  
  global cond_tolerance
  global pairx
  global pairy
  global pairid
  global pairx_store
  global pairy_store
  global pairid_store 
  global pair_flag tftree
  
  global robot_pub_vel robot_pub_msg target_pub_vel target_pub_msg

  
  N=6;
 %boundary
 %vertex=[2.86 10.5;-3.55 1.15;-2.33 -8.28;7.49 -0.8];
% vertex=[2.86 10.5;-3.55 1.15;-2.49 -8.6;7.57 -0.8];
vertex=[2.86 10.5;-3.55 1.15;-2.52 -8.77;7.57 -0.8];
 %vertex=[2.71 10.1;-3.55 1.15;-2.24 -8.01;7.23 -0.688];
 %vertex=[2.73 10.5;-3.55 1.15;-2.28 -7.97;7.15 -0.709];
 
 %vertex=[2.73 10.3;-3.42 1.22;-2.16 -7.72;7.03 -0.648];
 
 cornor_epsilon=0.5;   
 %linear_vel_max=0.25;
 omega_max=10*pi/180;
 
 k_wtov=0.025*180/pi;
 angle_kp=0.3;
    
%  tarx_hat=[1.25, 1.25, 1.25,1.25];
%  tary_hat=[0.39, 0.39, 0.39,0.39];
%  
%  tarxtrue=1.25;
%  tarytrue=0.39;
 
%  tarx_hat=[1.62, 1.62, 1.62, 1.62];
%  tary_hat=[-0.566, -0.566, -0.566, -0.566];
%  
 tarxtrue=0;
 tarytrue=0;
 tarx_hat=[0.5, 0.5, 0.5, 0.5, 0.5, 0.5]';
 tary_hat=[-0.5, -0.5, -0.5, -0.5, -0.5, -0.5]';
 
%  tarxtrue=1.58;
%  tarytrue=-0.285;

 

 
 % counterclockwise order is +, define the edge, P->Q->R->S->P
 edge=[vertex(1,:) vertex(2,:);...
             vertex(2,:) vertex(3,:);...
             vertex(3,:) vertex(4,:);...
             vertex(4,:) vertex(1,:);...
             ];
         
 % initial edge that robot is on, i_edge_flag=zeros(1,N)
i_edge_flag=ones(1,N);
%i_edge_last=zeros(1,N);

% all the robots are located at the e1 and firstly regard the left vertex
% of e1 as the cornor goal
%vd=[vertex(2,:);vertex(2,:);vertex(2,:);];
vd=zeros(N,2);

%roboti_vd_angle=zeros(1,N);

%comm_flag=ones(1,N);

i_store=zeros(N,2); %store the info for its two neighbors

T=0.5;

iself_count=ones(1,N);

%self_tolerance=omega_max/2+0.1;
self_tolerance=0.2;

gVimid=zeros(1,N);
%errp=zeros(1,N);
ave_err=[];
ave_com=[];
con_pi_err=[];

com_time=zeros(1,N);

timer_cnt=1;

Sigma_hat=[eye(2);eye(2);eye(2);eye(2);eye(2);eye(2)];

id=zeros(N,1);

%pose_x_store=zeros(3*N,1);
%pose_y_store=zeros(3*N,1);


cond_tolerance=2;
pairx=zeros(N,1);
pairy=zeros(N,1);
pairid=zeros(N,1);
pairx_store=zeros(N,1);
pairy_store=zeros(N,1);
pairid_store=zeros(N,1);
pair_flag=1;


robot_0_pub_vel = rospublisher('/robot_0/cmd_vel');
robot_1_pub_vel = rospublisher('/robot_1/cmd_vel');
robot_2_pub_vel = rospublisher('/robot_2/cmd_vel');
robot_3_pub_vel = rospublisher('/robot_3/cmd_vel');
robot_4_pub_vel = rospublisher('/robot_4/cmd_vel');
robot_5_pub_vel = rospublisher('/robot_5/RosAria/cmd_vel'); % robot 5 is a real pioneer
%robot_5_pub_vel = rospublisher('/robot_5/cmd_vel');

robot_pub_vel=[robot_0_pub_vel;robot_1_pub_vel;robot_2_pub_vel;robot_3_pub_vel;...
    robot_4_pub_vel;robot_5_pub_vel];
robot_pub_msg = rosmessage('geometry_msgs/Twist');
% publish target vel_cmd
target_pub_vel = rospublisher('/robot_6/RosAria/cmd_vel');
target_pub_msg = rosmessage('geometry_msgs/Twist');


%subscribe the pose topic to get the positon if the robot
tftree = rostf;
waitForTransform(tftree, '/global_origin', '/robot_0/base_link');
waitForTransform(tftree, '/global_origin', '/robot_1/base_link');
waitForTransform(tftree, '/global_origin', '/robot_2/base_link');
waitForTransform(tftree, '/global_origin', '/robot_3/base_link');
waitForTransform(tftree, '/global_origin', '/robot_4/base_link');
waitForTransform(tftree, '/global_origin', '/robot_5/base_link');
waitForTransform(tftree, '/global_origin', '/robot_6/base_link');
%posesub_tar = rossubscriber('/robot_6/RosAria/pose');
%
%
t = timer('TimerFcn',{@fivesim_tworeal_disself_ICI},...
    'Period',T,'ExecutionMode','fixedSpacing');
start(t);