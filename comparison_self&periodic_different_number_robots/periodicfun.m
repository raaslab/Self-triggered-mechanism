function [convtime,consumswitch] = periodicfun(N,initheta,convthresh)

T=0.1; % sampling period
max_step=1600;
omega_max=1; %maximum angular velocity for each sensor agent

itheta=zeros(N,max_step); %location 
theta=zeros(N+2,max_step);
phi=zeros(N,max_step);%relative angular distance
P=zeros(N,max_step); %communicaiton power
Con=zeros(N,max_step); % convergence speed
Vimid=zeros(N,max_step); %the midepoint of i's Voronoi set 
pretemp=zeros(N,max_step);%store the previous step

sigu=zeros(N,max_step); %the direction of robot(clockwise & counterclockwise) 
switchsign=zeros(N,max_step); %the change of directions
sumswitch=zeros(1,max_step);%direction switch per step per robot


%itheta(:,1)=[20;60;80;130;200;250;278;290;311;330];
%itheta(:,1)=[20;80;130;200;250;330];% initial locations of six sensor agents, counterclosewise order
itheta(:,1)=initheta;
theta(:,1)=[(itheta(N,1)-360);itheta(:,1);(itheta(1,1)+360)]; %virtual agent 0th:=agent N-2pi;virtual agent 7th:=agent 1st+ 2pi

for i=1:N
    Vimid(i,1)=1/4*(theta(i+2,1)+2*theta(i+1,1)+theta(i,1));   
    sigu(i,1)=sign(theta(i+2,1)-2*theta(i+1,1)+theta(i,1));
    pretemp(i,1)=sigu(i,1);
    
    u=min(omega_max,abs(1/4*(theta(i+2,1)-2*theta(i+1,1)+theta(i,1))));
    itheta(i,2)=itheta(i,1)+T*sigu(i,1)*u; 
end 
theta(:,2)=[(itheta(N,2)-360);itheta(:,2);(itheta(1,2)+360)]; 

for k=2:max_step    
   for i=1:N
        Vimid(i,k)=1/4*(theta(i+2,k)+2*theta(i+1,k)+theta(i,k));   
        sigu(i,k)=sign(theta(i+2,k)-2*theta(i+1,k)+theta(i,k));
%         if (sigu(i,k)*sigu(i,k-1)<0)xx
%             switchsign(i,k)=switchsign(i,k-1)+1;
%         else 
%             switchsign(i,k)=switchsign(i,k-1);
%         end
        if (sigu(i,k)*pretemp(i,k-1)~=0)
            if(sigu(i,k)*pretemp(i,k-1)<0)
                pretemp(i,k)=sigu(i,k);
                switchsign(i,k)=switchsign(i,k-1)+1;
            else     
                pretemp(i,k)=pretemp(i,k-1);
                switchsign(i,k)=switchsign(i,k-1);
            end
            
        else 
            if(sigu(i,k)~=0)
            pretemp(i,k)=sigu(i,k);
            else
            pretemp(i,k)=pretemp(i,k-1);   
            end
         switchsign(i,k)=switchsign(i,k-1);
       end
        u=min(omega_max,abs(1/4*(theta(i+2,k)-2*theta(i+1,k)+theta(i,k))));
        itheta(i,k+1)=itheta(i,k)+T*sigu(i,k)*u; 
   end
   sumswitch(k)=sum(switchsign(:,k))/N;
   theta(:,k+1)=[(itheta(N,k+1)-360);itheta(:,k+1);(itheta(1,k+1)+360)]; 

end
 


for k=1:max_step
for i=1:N
    phi(i,k)=theta(i+1,k)-theta(i,k);
    P(i,k)=log10(10^(0.1+abs(theta(i+1,k)-theta(i,k)))+10^(0.1+abs(theta(i+2,k)-theta(i+1,k))));
    %Con(i,k)=abs(phi(i,k)-360/N);
    Con(i,k)=abs(itheta(i,k)-Vimid(i,k));
end 
end


% for i=1:N
%    %plot(itheta(i,:))
%     plot(phi(i,:))
%     hold on
% end

%plot(sum(P)), hold on
plot(sum(Con)),hold on

convtime = find(sum(Con)<convthresh,1,'first');
%consumswitch=sumswitch(convtime)/convtime;
consumswitch=sumswitch(convtime);
end