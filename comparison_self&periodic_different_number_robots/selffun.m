function [convtime,selfcomessage,consumswitch,consumabsu,sumwongdir] = selffun(N,initheta,convthresh)
%function [convtime,sumctime] = selffun(N,initheta,convthresh)
%N=6; %the number of sensor agents 
T=0.5; % sampling period
max_step=2000;
omega_max=1; %maximum angular velocity for each sensor agent
delta=0.1;


itheta=zeros(N,max_step); %locations of all agents at all timesteps 
theta=zeros(N+2,max_step);
phi=zeros(N,max_step);%relative angular distance
P=zeros(N,max_step); %communicaiton power
Con=zeros(N,max_step); % convergence speed
gVimid=zeros(N,max_step); %the midepoint of i's guaranteed Voronoi set 

u=zeros(N,max_step);
absu=zeros(N,max_step); %the abs value of the control input
pretemp=zeros(N,max_step);%store the previous step
%sigu=zeros(N,max_step); %the dection of the robot(clockwise & counterclockwise) 
switchsign=zeros(N,max_step); %the change of directions
sumswitch=zeros(1,max_step);%direction switch per step per robot
sumabsu=zeros(1,max_step);%abs velocity value per step per robot


itheta(:,1)=initheta;
%itheta(:,1)=round(sort(360*rand(N,1)));% initial locations of six sensor agents, counterclosewise order
%temptheta=round(sort(360*rand(N,1)));
%itheta(:,1)=temptheta;
theta(:,1)=[(itheta(N,1)-360);itheta(:,1);(itheta(1,1)+360)]; %virtual agent 0th:=agent N-2pi;virtual agent 7th:=agent 1st+ 2pi

R=zeros(2*N, max_step); %information agent keeps for its neighbors
for i=1:N
    R(2*i-1:2*i,2)=[theta(i,1);(theta(i+2,1))];
    utemp=1/4*(theta(i+2,1)-2*theta(i+1,1)+theta(i,1));
    pretemp(i,1)=sign(utemp);
    u(i,1)=sign(utemp)*min(omega_max,abs(utemp));
    %itheta(i,2)=itheta(i,1)+1/4*T*(theta(i+2,1)-2*theta(i+1,1)+theta(i,1));% at the initial step, agent knows the exact info. of its meihhbors.
    itheta(i,2)=itheta(i,1)+T*u(i,1);
    absu(i,1)=abs(u(i,1));
    gVimid(i,1)=1/4*(theta(i+2,1)+2*theta(i+1,1)+theta(i,1));
end

sumabsu(1,1)=sum(absu(:,1))/N;
theta(:,2)=[(itheta(N,2)-360);itheta(:,2);(itheta(1,2)+360)];

C=zeros(N,max_step);%communication record;
Ctime=zeros(N,max_step);%communication times
sumctime=zeros(1,max_step);%communication times at each step per robot



C(:,1)=1;
Ctime(:,1)=1;
count=ones(N,1);
sumctime(1,1)=1;

for k=2: max_step
   
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
             C(i,k)=1;%if couumicaiton occurs, set the flag 1
             Ctime(i,k)=Ctime(i,k-1)+1;
             count(i)=1; % reset the number of omega*T
           
             %u(i,k)=1/4*(theta(i+2,k)-2*theta(i+1,k)+theta(i,k));
             utemp=1/4*(theta(i+2,k)-2*theta(i+1,k)+theta(i,k));
             u(i,k)=sign(utemp)*min(omega_max,abs(utemp));
             itheta(i,k+1)=itheta(i,k)+T*u(i,k);
             %itheta(i,k+1)=itheta(i,k)+1/4*T*(theta(i+2,k)-2*theta(i+1,k)+theta(i,k));
            
          else
              
             R(2*i-1:2*i,k+1)=R(2*i-1:2*i,k); %keep memory
             Ctime(i,k)=Ctime(i,k-1);
             count(i)=count(i)+1;
             
                 if (abserrp>=ubdi+omega_max*T)
                 u(i,k)=omega_max*errp/abserrp;
                 itheta(i,k+1)=itheta(i,k)+T*omega_max*errp/abserrp;
                 elseif (abserrp<=ubdi)
                 u(i,k)=0;
                 itheta(i,k+1)=itheta(i,k);
                 else
                 u(i,k)=(abserrp-ubdi)/T*errp/abserrp;   
                 itheta(i,k+1)=itheta(i,k)+T*(abserrp-ubdi)/T*errp/abserrp; 
                 end
                 
          end    
%                if (sign(u(i,k))*sign(u(i,k-1))<0)
%                    switchsign(i,k)=switchsign(i,k-1)+1;
%                else 
%                    switchsign(i,k)=switchsign(i,k-1);
%                end
        absu(i,k)=abs(u(i,k));
        if (sign(u(i,k))*pretemp(i,k-1)~=0)
            if(sign(u(i,k))*pretemp(i,k-1)<0)
                pretemp(i,k)=sign(u(i,k));
                switchsign(i,k)=switchsign(i,k-1)+1;
            else     
                pretemp(i,k)=pretemp(i,k-1);
                switchsign(i,k)=switchsign(i,k-1);
            end
            
        else 
            if(sign(u(i,k))~=0)
            pretemp(i,k)=sign(u(i,k));
            else
            pretemp(i,k)=pretemp(i,k-1);   
            end
        switchsign(i,k)=switchsign(i,k-1);
       end

    end
    sumctime(k)=sum(Ctime(:,k))/N;
    sumswitch(k)=sum(switchsign(:,k))/N;
    sumabsu(k)=sum(absu(:,k))/N;
    theta(:,k+1)=[(itheta(N,k+1)-360);itheta(:,k+1);(itheta(1,k+1)+360)];
    
    % Plot the robots
%     figure(1); cla; hold on; axis equal
%     th = 0 : 0.1 : 2*pi;
%     cx = cos(th); cy = sin(th);
%     plot(cx,cy,'--');
%     
%     for i = 1 : size(itheta,1)
%         plot(cos(deg2rad(itheta(i,k))), sin(deg2rad(itheta(i,k))),'ro','MarkerFaceColor','r');
%     end
%     pause(0.01);
end


for k=1:max_step
for i=1:N
    phi(i,k)=theta(i+1,k)-theta(i,k);
    P(i,k)=C(i,k)*log10(10^(0.1+abs(theta(i+1,k)-theta(i,k)))+10^(0.1+abs(theta(i+2,k)-theta(i+1,k))));
    %Con(i,k)=abs(phi(i,k)-360/N);
    Con(i,k)=abs(itheta(i,k)-gVimid(i,k));
end 
end

% for i=1:N
%     %plot(itheta(i,:)),hold on
%     %plot(phi(i,:)), hold on
%     plot(C(i,:)), hold on
% end

%plot(sum(P)),hold on
plot(sum(Con)), hold on

% figure; hold on;
% for i = 1 : N
% A=find(C(i,:)>0);plot(A,i*ones(length(A)),'ro')
% end

convtime = find(sum(Con)<convthresh,1,'first');
selfcomessage=sumctime(convtime)/convtime;
%consumswitch=sumswitch(convtime)/convtime;
consumswitch=sumswitch(convtime);
consumabsu=sumabsu(convtime)/convtime;

wrongdir=zeros(N,max_step);
sumwrongd=zeros(1,max_step);

for i=1:N
     rightdir=itheta(i,convtime)-itheta(i,1);
     if (u(i,1)*rightdir<0)
        wrongdir(i,1)=1;
    else
        wrongdir(i,1)=0;
     end
%     phi(i,1)=theta(i+1,1)-theta(i,1);
%     P(i,1)=log10(10^(0.1+abs(theta(i+1,1)-theta(i,1)))+10^(0.1+abs(theta(i+2,1)-theta(i+1,1))));
%     %Con(i,k)=abs(phi(i,k)-360/N);
%     Con(i,1)=abs(itheta(i,1)-gVimid(i,1));
end
    sumwrongd(1)=sum(wrongdir(:,1))/N;
    

for k=2:max_step
    for i=1:N
        rightdir=itheta(i,convtime)-itheta(i,1);
        if (u(i,k)*rightdir<0)
            wrongdir(i,k)=wrongdir(i,k-1)+1;
        else
            wrongdir(i,k)=wrongdir(i,k-1);
        end
%         phi(i,k)=theta(i+1,k)-theta(i,k);
%         P(i,k)=log10(10^(0.1+abs(theta(i+1,k)-theta(i,k)))+10^(0.1+abs(theta(i+2,k)-theta(i+1,k))));
%         %Con(i,k)=abs(phi(i,k)-360/N);
%         Con(i,k)=abs(itheta(i,k)-gVimid(i,k));
    end 
    sumwrongd(k)=sum(wrongdir(:,k))/N;
end
sumwongdir=sumwrongd(convtime);
% 
% 
end