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
  global tarx_hat
  global tary_hat
  global vd % the next vertex robot is going to reach;
  %global i_edge_last;
  %global roboti_vd_angle;
  %global comm_flag;
  global i_store
  global iself_count 
  global self_tolerance
  global gVimid
  
  global timer_cnt
  global Sigma_hat
  global id % update the EKF filter
  
   global ave_err ave_com
  
   global first_com
   
  
  N=4;
 %boundary
 vertex=[2.86 10.5;-3.55 1.15;-2.43 -8.68;7.23 -0.688];
 
 cornor_epsilon=0.6;   
 %linear_vel_max=0.25;
 omega_max=10*pi/180;
 
 k_wtov=0.02*180/pi;
 angle_kp=0.12;
    
 tarx_hat=1.25;
 tary_hat=0.39;
 
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

iself_count=zeros(1,N);

self_tolerance=omega_max/2+0.1;

gVimid=zeros(1,N);
ave_err=[];
ave_com=[];

timer_cnt=1;

Sigma_hat=eye(2);

id=zeros(N,1);

first_com=1;

t = timer('TimerFcn',{@fourrobot_self_movtar_cenEKF},...
    'Period',T,'ExecutionMode','fixedSpacing');
start(t);
