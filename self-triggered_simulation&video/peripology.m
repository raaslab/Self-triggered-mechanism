N=6; %the number of sensor agents 
T=0.1; % sampling period
max_step=800;
omega_max=1; %maximum angular velocity for each sensor agent
error_mat = zeros(1, max_step);
%cross point of the boundary
P=[3;7];
Q=[8;17]; 
R=[5;26]; 
S=[1;22];
Tar=[4;20];% jointed positions and the target

px=zeros(N,max_step);
py=zeros(N,max_step);

itheta=zeros(N,max_step); %location 
theta=zeros(N+2,max_step);
phi=zeros(N,max_step);%relative angular distance
Pcost=zeros(N,max_step); %communicaiton power
Con=zeros(N,max_step); % convergence speed
Vimid=zeros(N,max_step); %the midepoint of i's Voronoi set 

%itheta(:,1)=[20;60;80;130;200;250;278;290;311;330];
itheta(:,1)=[69.8979;75.0730;83.1024;108.1016;170.1016;303.8982];
%itheta(:,1)=[20;80;130;200;250;330];% initial locations of six sensor agents, counterclosewise order
%itheta(:,1)=round(sort(360*rand(N,1)));

% set the initial point on the line y=2x+1
% px(:,1)=sort(P(1,1) + (Q(1,1)-P(1,1)).*rand(N,1));
% py(:,1)=2*px(:,1)+1; %y=2x+1

for i=1:N
    %itheta(i,1)=positiontoangularfun(py(i,1)-Tar(2,1),px(i,1)-Tar(1,1));
    [px(i,1),py(i,1)]=angulartopositionfun(itheta(i,1),Tar(1,1),Tar(2,1));
end

theta(:,1)=[(itheta(N,1)-360);itheta(:,1);(itheta(1,1)+360)]; %virtual agent 0th:=agent N-2pi;virtual agent 7th:=agent 1st+ 2pi

%Set up the movie.
writerObj = VideoWriter('peri_static_tracking.avi'); % Name it.
writerObj.FrameRate = 60; % How many frames per second.
open(writerObj); 

for k=1:max_step
       
   for i=1:N
    Vimid(i,k)=1/4*(theta(i+2,k)+2*theta(i+1,k)+theta(i,k));
    Con(i,k)=abs(itheta(i,k)-Vimid(i,k));
    sigu=sign(theta(i+2,k)-2*theta(i+1,k)+theta(i,k));
    u=min(omega_max,abs(1/4*(theta(i+2,k)-2*theta(i+1,k)+theta(i,k))));
    itheta(i,k+1)=itheta(i,k)+T*sigu*u; 
    [px(i,k),py(i,k)]=angulartopositionfun(itheta(i,k), Tar(1,1), Tar(2,1));
   end
 
   theta(:,k+1)=[(itheta(N,k+1)-360);itheta(:,k+1);(itheta(1,k+1)+360)]; 
 
% %/////////     
% %Plot the robots
%figure(1);
subplot(2,2,1)
%title('Unit circle','fontsize',12)
    xlabel({'$$x$$'},'Interpreter','latex','fontsize',14)
    ylabel({'$$y$$'},'Interpreter','latex','fontsize',14)
%     figure(1); 
    cla; hold on; axis equal
    th = 0 : 0.1 : 2*pi;
    cx = cos(th); cy = sin(th);
    plot(cx,cy,'--');

    
    for i = 1 : size(itheta,1)
%          figure(1);
        plot(0,0,'*');hold on
%          figure(1);
        plot(cos(deg2rad(itheta(i,k))), sin(deg2rad(itheta(i,k))),'ro','MarkerFaceColor','r');
        plot ([cos(deg2rad(itheta(i,k))) 0], [sin(deg2rad(itheta(i,k))) 0],':')
    end
   
    pause(0.01);
% %//////////

%----------------------------------------------------
%figure(1);
subplot(2,2,2);
%title('Polygon','fontsize',12)
    xlabel({'$$x$$'},'Interpreter','latex','fontsize',14)
    ylabel({'$$y$$'},'Interpreter','latex','fontsize',14)
% figure(2); 
cla; hold on;axis equal
%plot the boundary
x1=3:0.1:8;
y1=2*x1+1; %line 1
plot(x1,y1)
hold on

x2=5:0.1:8;
y2=-3*x2+41; %line 2
%  figure(2);
plot(x2,y2) 
hold on

x3=1:0.1:5;
y3=x3+21; %line 3
%  figure(2);
plot(x3,y3) 
hold on 

x4=1:0.1:3;
y4=-7.5*x4+29.5; %line 4
%  figure(2);
plot(x4,y4)
hold on
 
% figure(2);
plot(4,20,'*'), hold on

    for i = 1 : size(itheta,1)
%        figure(2);
        plot(px(i,k), py(i,k),'ro','MarkerFaceColor','r'); hold on 
        plot ([px(i,k) Tar(1,1)], [py(i,k) Tar(2,1)],':')
       
    end
    pause(0.01);
 
%figure(1);
subplot(2,2,[3,4])
title('Periodic tracking with a stationary target','fontsize',14)
    xlabel({'$$k$$'},'Interpreter','latex','fontsize',14)
    %ylabel({'$$y$$'},'Interpreter','latex','fontsize',14)
    ylabel('Convererr','fontsize',14)
%     figure(1); 
    
    %cla;
 % axis equal
  %  plot(1000,0);
    cla;hold on;
    plot(800, 65);
    hold on;
    
    plot(800, 0);
    hold on;
    error_mat(1,k) = sum(Con(:,k));
    plot(1:k, error_mat(:,1:k),'r');
    %cla;hold on;
%%--------------------------------------------------
   %if mod(i,4)==0, % Uncomment to take 1 out of every 4 frames.
        frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
        writeVideo(writerObj, frame);
    %end

end
hold  off
close(writerObj); % Saves the movie.

% for k=1:max_step
% for i=1:N
%     phi(i,k)=theta(i+1,k)-theta(i,k);
%     Pcost(i,k)=log10(10^(0.1+abs(theta(i+1,k)-theta(i,k)))+10^(0.1+abs(theta(i+2,k)-theta(i+1,k))));
%     %Con(i,k)=abs(phi(i,k)-360/N);
%     Con(i,k)=abs(itheta(i,k)-Vimid(i,k));
% end 
% end

% for i=1:N
%    %plot(itheta(i,:))
%     plot(phi(i,:))
%     hold on
% end

%plot(sum(P)), hold on
%plot(sum(Con)),hold on


