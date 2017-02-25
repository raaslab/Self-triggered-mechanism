N=6; %the number of sensor agents 
T=0.1; % sampling period
max_step=2000;
omega_max=1; %maximum angular velocity for each sensor agent
delta=0.06;
cond_tolerance=1;

px=zeros(N,max_step);
py=zeros(N,max_step);
pairx=zeros(N,max_step);
pairy=zeros(N,max_step);
pairid=zeros(N,max_step);
%considering the head and the tail, we enlarge the state 
%head_px_tail=zeros(N+2, max_step);
%head_py_tail=zeros(N+2, max_step);

% true postion of target, it is specificed at one step
tarxtrue=zeros(1,max_step);
tarytrue=zeros(1,max_step); 

%estimation of the target
tarx_hat=zeros(N,max_step);
tary_hat=zeros(N,max_step);
%the covariance funciton 
Sigma_hat=zeros(2*N,2*max_step);
id=zeros(N,1);


for i=1:N
tarx_hat(i,1)=4;
tary_hat(i,1)=20;
Sigma_hat((2*i-1):2*i,1:2)=[1 0; 0 1];
end
% it is updated as the time passing by

itheta=zeros(N,max_step); %locations of all agents at all timesteps 
theta=zeros(N+2,max_step);
phi=zeros(N,max_step);%relative angular distance
P=zeros(N,max_step); %communicaiton power
Con=zeros(N,max_step); % convergence speed
gVimid=zeros(N,max_step); %the midepoint of i's guaranteed Voronoi set
u=zeros(N,max_step);

itheta(:,1)=[69.8979;75.0730;83.1024;108.1016;170.1016;303.8982];
%itheta(:,1)=round(sort(360*rand(N,1)));% initial locations of six sensor agents, counterclosewise order
theta(:,1)=[(itheta(N,1)-360);itheta(:,1);(itheta(1,1)+360)]; %virtual agent 0th:=agent N-2pi;virtual agent 7th:=agent 1st+ 2pi


%initial states obtained from mapping back the angualr
for i=1:N
    id(i,1)=i;
    %itheta(i,1)=positiontoangularfun(py(i,1)-20,px(i,1)-4);
    [px(i,1),py(i,1)]=angulartopositionfun(itheta(i,1),tarx_hat(i,1),tary_hat(i,1));
end

idb=[N;id(:,1);1];
head_px_tail(:,1)=[px(N,1);px(:,1);px(1,1)];
head_py_tail(:,1)=[py(N,1);py(:,1);py(1,1)];

%update the estamate and covriance
%function [x_hat_tplus1, y_hat_tplus1, Sigma_hat_tplus1, x_true, y_true] = KF (...x_hat_t, y_hat_t, Sigma_hat_t, Xr, Yr, t)
% for i=1:N
% 
% end
R=zeros(2*N, max_step); %information agent keeps for its neighbors /angular
pxstore=zeros(3*N, max_step);%information of x agent keeps for its neighbors /position
pystore=zeros(3*N, max_step);%information of y agent keeps for its neighbors /position
for i=1:N
    %calculate the best pair for each robot at the first time step
    
    [pairx(i,1),pairy(i,1),pairid(i,1)]=censer_condpair(N,px(i,1),py(i,1),px,py,tarx_hat(i,1),tary_hat(i,1));
    
    
    %just store the info at last step
    R(2*i-1:2*i,2)=[theta(i,1);(theta(i+2,1))];
    pxstore((3*i-2):3*i,2)=head_px_tail(i:(i+2),1);
    pystore((3*i-2):3*i,2)=head_py_tail(i:(i+2),1);
    
    gVimid(i,1)=1/4*(theta(i+2,1)+2*theta(i+1,1)+theta(i,1));
    utemp=1/4*(theta(i+2,1)-2*theta(i+1,1)+theta(i,1));
    %pretemp(i,1)=sign(utemp);
    u(i,1)=sign(utemp)*min(omega_max,abs(utemp));
    %itheta(i,2)=itheta(i,1)+1/4*T*(theta(i+2,1)-2*theta(i+1,1)+theta(i,1));% at the initial step, agent knows the exact info. of its meihhbors.
    itheta(i,2)=itheta(i,1)+T*u(i,1);
   
    %/********/target input-----------
    [px(i,2),py(i,2)]=angulartopositionfun(itheta(i,2),tarx_hat(i,1),tary_hat(i,1));
     %itheta(i,2)=itheta(i,1)+1/4*T*(theta(i+2,1)-2*theta(i+1,1)+theta(i,1));% at the initial step, agent knows the exact info. of its meihhbors.
    [tarx_hat(i,2), tary_hat(i,2), Sigma_hat((2*i-1):2*i, (2*2-1):2*2), tarxtrue(1,1), tarytrue(1,1)]=KF (...
    tarx_hat(i,1),tary_hat(i,1), Sigma_hat((2*i-1):2*i,1:2), head_px_tail(i:(i+2),1), head_py_tail(i:(i+2),1), idb(i:(i+2)), 1);
  %the orientation based on the target at current step
   itheta(i,2)=positiontoangularfun((py(i,2)-tary_hat(i,2)),(px(i,2)-tarx_hat(i,2)));       

end

head_px_tail(:,2)=[px(N,2);px(:,2);px(1,2)];
head_py_tail(:,2)=[py(N,2);py(:,2);py(1,2)];
itheta(:,2)=orderlythetafun(itheta(:,2));
theta(:,2)=[(itheta(N,2)-360);itheta(:,2);(itheta(1,2)+360)];

C=zeros(N,max_step);%communication record;
C(:,1)=1;
count=ones(N,1);

% %Set up the movie.
% writerObj = VideoWriter('selfKFdecen.avi'); % Name it.
% writerObj.FrameRate = 60; % How many frames per second.
% open(writerObj); 

for k=2: max_step
   %[tarx_hat(1,k+1), tary_hat(1,k+1), Sigma_hat(1:2, (2*(k+1)-1):(2*(k+1))), tarxtrue(1,k), tarytrue(1,k)] = KF (...
%tarx_hat(1,k), tary_hat(1,k), Sigma_hat(1:2,(2*k-1): 2*k), px(:,k), py(:,k), k);
    
    for i=1:N
         
          ubdi=omega_max*T*count(i)/2; %upper bound
          r=omega_max*T*count(i); % the prediciton range of neighbors' motion
          gVimid(i,k)=1/4*(R(2*i,k)+2*itheta(i,k)+R(2*i-1,k)); % the midepoint of i's guaranteed Voronoi set 
          
          errp=gVimid(i,k)-itheta(i,k); 
          abserrp=abs(gVimid(i,k)-itheta(i,k));
          proximity=max(abserrp,delta); % the degree agent goes towards midpoint of its guaranteed Voronoi set
         
          if ((ubdi>=proximity)||(R(2*i,k)-r<=itheta(i,k))||(R(2*i-1,k)+r>=itheta(i,k)))
          %if (ubdi>=proximity)    
             R(2*i-1:2*i,k+1)=[theta(i,k);(theta(i+2,k))];% update the stored memory of its neighbors
             pxstore((3*i-2):3*i,k+1)=head_px_tail(i:(i+2),k);
             pystore((3*i-2):3*i,k+1)=head_py_tail(i:(i+2),k);
             
             %pairx(i,1),pairy(i,1),pairid(i,1)
             pairx(i,k)=pairx(i,k-1);
             pairy(i,k)=pairy(i,k-1);
             pairid(i,k)=pairid(i,k-1);
             
             C(i,k)=1;%if couumicaiton occurs, set the flag 1
             count(i)=1; % reset the number of omega*T
           
             %itheta(i,k+1)=itheta(i,k)+1/4*T*(theta(i+2,k)-2*theta(i+1,k)+theta(i,k));
             utemp=1/4*(theta(i+2,k)-2*theta(i+1,k)+theta(i,k));
             u(i,k)=sign(utemp)*min(omega_max,abs(utemp));
             itheta(i,k+1)=itheta(i,k)+T*u(i,k);
             %update estimate of target if acquire new info from its neighbors 
             [tarx_hat(i,k+1), tary_hat(i,k+1), Sigma_hat((2*i-1):2*i, (2*(k+1)-1):(2*(k+1))), tarxtrue(1,k), tarytrue(1,k)] = KF (...
tarx_hat(i,k), tary_hat(i,k), Sigma_hat((2*i-1):2*i,(2*k-1): 2*k), pxstore((3*i-2):3*i,k+1), pystore((3*i-2):3*i,k+1), idb(i:(i+2)), k);
             
            %the orientation based on the target at last step
            [px(i,k+1),py(i,k+1)]=angulartopositionfun(itheta(i,k+1),tarx_hat(i,k), tary_hat(i,k));
            %the orientation based on the target at current step
            itheta(i,k+1)=positiontoangularfun((py(i,k+1)-tary_hat(i,k+1)),(px(i,k+1)-tarx_hat(i,k+1)));
          else
              
             R(2*i-1:2*i,k+1)=R(2*i-1:2*i,k); %keep memory
             
             % the central server decide when to update the best pair
             if  condobserva(tarx_hat(i,k), tary_hat(i,k), pairx(i,k-1),pairy(i,k-1),px(i,k),py(i,k))>cond_tolerance
                 % choose the best pair
                 [pairx(i,k),pairy(i,k),pairid(i,k)]=censer_condpair(N,px(i,k),py(i,k),px(:,k),py(:,k),tarx_hat(i,k),tary_hat(i,k));
             else %keep still
                 pairx(i,k)=pairx(i,k-1);
                 pairy(i,k)=pairy(i,k-1);
                 pairid(i,k)=pairid(i,k-1);
             end
             %agent knows info about itself, but keeps the info of its neighbors 
             %we should give the best prediciton about the future state of
             %its neighbors, it is the map from set to fixed point, so its
             %should be updated 
             %pxstore((3*i-2):3*i,k+1)=[pxstore((3*i-2),k);px(i,k);pxstore(3*i,k)];
             %pystore((3*i-2):3*i,k+1)=[pystore((3*i-2),k);py(i,k);pystore(3*i,k)];
%              [tarx_hat(i,k+1), tary_hat(i,k+1), Sigma_hat((2*i-1):2*i, (2*(k+1)-1):(2*(k+1))), tarxtrue(1,k), tarytrue(1,k)] = KF (...
% tarx_hat(i,k), tary_hat(i,k), Sigma_hat((2*i-1):2*i,(2*k-1): 2*k), pxstore((3*i-2):3*i,k+1), pystore((3*i-2):3*i,k+1), k);
              [tarx_hat(i,k+1), tary_hat(i,k+1), Sigma_hat((2*i-1):2*i, (2*(k+1)-1):(2*(k+1))), tarxtrue(1,k), tarytrue(1,k)] = KF (...
tarx_hat(i,k), tary_hat(i,k), Sigma_hat((2*i-1):2*i,(2*k-1): 2*k), [px(i,k);pairx(i,k)], [py(i,k);pairy(i,k)], [id(i),pairid(i,k)], k);
             
             count(i)=count(i)+1;
             
                 if (abserrp>=ubdi+omega_max*T)
                 itheta(i,k+1)=itheta(i,k)+T*omega_max*errp/abserrp;
                  [px(i,k+1),py(i,k+1)]=angulartopositionfun(itheta(i,k+1),tarx_hat(i,k), tary_hat(i,k));
                  itheta(i,k+1)=positiontoangularfun((py(i,k+1)-tary_hat(i,k+1)),(px(i,k+1)-tarx_hat(i,k+1)));
                 elseif (abserrp<=ubdi)
                 itheta(i,k+1)=itheta(i,k);
                  [px(i,k+1),py(i,k+1)]=angulartopositionfun(itheta(i,k+1),tarx_hat(i,k), tary_hat(i,k));
                  itheta(i,k+1)=positiontoangularfun((py(i,k+1)-tary_hat(i,k+1)),(px(i,k+1)-tarx_hat(i,k+1)));
                 else
                 itheta(i,k+1)=itheta(i,k)+T*(abserrp-ubdi)/T*errp/abserrp; 
                  [px(i,k+1),py(i,k+1)]=angulartopositionfun(itheta(i,k+1),tarx_hat(i,k), tary_hat(i,k));
                  itheta(i,k+1)=positiontoangularfun((py(i,k+1)-tary_hat(i,k+1)),(px(i,k+1)-tarx_hat(i,k+1)));
                 end
                 
          end    
               
    end
    itheta(:,k+1)=orderlythetafun(itheta(:,k+1));
    theta(:,k+1)=[(itheta(N,k+1)-360);itheta(:,k+1);(itheta(1,k+1)+360)];
    head_px_tail(:,k+1)=[px(N,k+1);px(:,k+1);px(1,k+1)];
    head_py_tail(:,k+1)=[py(N,k+1);py(:,k+1);py(1,k+1)];
    
%     % Plot the robots
%     subplot(1,2,1)
%      cla; hold on; axis equal
%     th = 0 : 0.1 : 2*pi;
%     cx = cos(th); cy = sin(th);
%     plot(cx,cy,'--');
%     
%     for i = 1 : size(itheta,1)
%         plot(0,0,'*');hold on
%         plot(cos(deg2rad(itheta(i,k))), sin(deg2rad(itheta(i,k))),'ro','MarkerFaceColor','r'); hold on
%         plot ([cos(deg2rad(itheta(i,k))) 0], [sin(deg2rad(itheta(i,k))) 0],':')
%     end
%     pause(0.01);
%     
%      subplot(1,2,2)
% figure(2); 

%//****************************************************
figure(1);
title('Self-triggered tracking employing a decentralized EKF','fontsize',14)
    xlabel({'$$x$$'},'Interpreter','latex','fontsize',14)
    ylabel({'$$y$$'},'Interpreter','latex','fontsize',14)
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
%plot(4,20,'*'), hold on

    for i = 1 : size(itheta,1)
%        figure(2);
    %h(1) = covarianceEllipse([x_hat;y_hat],Sigma_hat,[1 0 0],11.82);
    %h(2) = plot(x_hat,y_hat,'rs','MarkerSize',8);
    %h(3) = plot(x_true,y_true,'bp','MarkerSize',8);
       h(1)=plot(px(i,k), py(i,k),'ro','MarkerFaceColor','r');
       h(2) = covarianceEllipse([tarx_hat(i,k);tary_hat(i,k)],Sigma_hat(2*i-1:2*i,(2*k-1):2*k),[1 0 0],11.82);
       h(3)=plot(tarx_hat(i,k),tary_hat(i,k),'rs','MarkerSize',8);
       h(4)=plot(tarxtrue(1,k),tarytrue(1,k),'bp','MarkerSize',8);
       h(5)=plot ([px(i,k) tarx_hat(i,k)], [py(i,k) tary_hat(i,k)],':');
    end
    pause(0.01);
%  %%//***********************************************************   
%  %if mod(i,4)==0, % Uncomment to take 1 out of every 4 frames.
%         frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
%         writeVideo(writerObj, frame);
%     %end
end


figure(2);
for k=1:max_step
for i=1:N
    %phi(i,k)=theta(i+1,k)-theta(i,k);
    %P(i,k)=C(i,k)*log10(10^(0.1+abs(theta(i+1,k)-theta(i,k)))+10^(0.1+abs(theta(i+2,k)-theta(i+1,k))));
    %Con(i,k)=abs(phi(i,k)-360/N);
    Con(i,k)=abs(itheta(i,k)-gVimid(i,k));
end 
end
% hold  off
% close(writerObj); % Saves the movie.
plot(sum(Con)),hold on