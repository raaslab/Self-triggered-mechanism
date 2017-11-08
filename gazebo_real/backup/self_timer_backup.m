%cross point of the boundary
% P=[-2.23;-8.13];
% Q=[7.16;-0.60];
% R=[2.73;9.96]; 
% S=[-3.38;1.21]; 
  global count;
  global vertex;

 count=1; 
 %vertex=[2.5 9.6;-3.84 1.29;-2.52 -8.71;7.6 -0.74];
 vertex=[2.68 10.9;-3.7 1.39;-2.56 -8.5;7.38 -0.688];

t = timer('TimerFcn',{@one_robot_navigation},...
    'Period',0.5,'ExecutionMode','fixedSpacing');
start(t);
