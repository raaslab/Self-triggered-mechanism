%cross point of the boundary
% P=[-2.23;-8.13];
% Q=[7.16;-0.60];
% R=[2.73;9.96]; 
% S=[-3.38;1.21];
  global N;
  global vertex;
  global edge;
  global i_edge_flag;
  global cornor_epsilon;
  global omega_max;
  global k_wtov; % simple transform from angle velocity (wrt to target) to the boundary linear_vel
  global angle_kp; %angular PID feedback parameter
  global tarx;
  global tary;
  global vd; % the next vertex robot is going to reach
  %global i_edge_last;
  %global roboti_vd_angle;
  
  global ave_com timer_cnt T

  T=0.5;
  N=3;
 %boundary
 vertex=[2.86 10.5;-3.55 1.15;-2.24 -8.01;7.23 -0.688];
 
 cornor_epsilon=0.6;   
 %linear_vel_max=0.25;
 omega_max=10*pi/180;
 
 k_wtov=0.02*180/pi;
 angle_kp=0.12;
    
 tarx=1.25;
 tary=0.39;
 
 % counterclockwise order is +, define the edge, P->Q->R->S->P
 edge=[vertex(1,:) vertex(2,:);...
             vertex(2,:) vertex(3,:);...
             vertex(3,:) vertex(4,:);...
             vertex(4,:) vertex(1,:);...
             ];
         
 % initial edge that robot is on, i_edge_flag=zeros(1,N)
i_edge_flag=[1 1 1];
%i_edge_last=zeros(1,N);

% all the robots are located at the e1 and firstly regard the left vertex
% of e1 as the cornor goal
%vd=[vertex(2,:);vertex(2,:);vertex(2,:);];
vd=zeros(N,2);

ave_com=[];
timer_cnt=1;

%roboti_vd_angle=zeros(1,N);

t = timer('TimerFcn',{@multi_robot_self},...
    'Period',T,'ExecutionMode','fixedSpacing');
start(t);

