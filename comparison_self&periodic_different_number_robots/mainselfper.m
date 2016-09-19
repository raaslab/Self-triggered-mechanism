clear all;      % clears all variables in your workspace
min_num_robots  = 3;
max_num_robots = 30;
num_trials = 30;
convthresh = 0.2;

for N = min_num_robots : max_num_robots
    for i = 1  : num_trials
        initheta=round(sort(360*rand(N,1)));
        cla;
        [selfconvtime(i,N),selfcomessage(i,N),selfsumswitch(i,N),selfsumabsu(i,N),selfsumwongdir(i,N)] = selffun(N,initheta,convthresh*N);
        %selfconvtime(i,N) =
        %selffun(N,initheta,convthresh*N);,selfsumwongdir(i,N),,persumwongdir(i,N)
        [perconvtime(i,N),persumswitch(i,N),persumabsu(i,N),persumwongdir(i,N)] = periodicfun(N,initheta,convthresh*N);
        initheta_all{N,i} = initheta;
    end
end


for N = min_num_robots : max_num_robots
avgperconvtime(N) = mean(perconvtime(:,N));
avgselfconvtime(N) = mean(selfconvtime(:,N));
stdselfconvtime(N) = std(selfconvtime(:,N));
stdperconvtime(N) = std(perconvtime(:,N));

avgpercomessage(N) = 1;
avgselfcomessage(N) = mean(selfcomessage(:,N));
stdpercomessage(N)=std(1);
stdselfcomessage(N)=std(selfcomessage(:,N));

avgpersumswitch(N)=mean(persumswitch(:,N));
avgselfsumswitch(N)=mean(selfsumswitch(:,N));
stdpersumswitch(N)=std(persumswitch(:,N));
stdselfsumswitch(N)=std(selfsumswitch(:,N));

avgpersumabsu(N) = mean(persumabsu(:,N));
avgselfsumabsu(N) = mean(selfsumabsu(:,N));
stdpersumabsu(N)=std(persumabsu(:,N));
stdselfsumabsu(N)=std(selfsumabsu(:,N));

 avgpersumwongdir(N) = mean(persumwongdir(:,N));
 avgselfsumwongdir(N) = mean(selfsumwongdir(:,N));
 stdpersumwongdir(N)=std(persumwongdir(:,N));
 stdselfsumwongdir(N)=std(selfsumwongdir(:,N));

end

figure; hold on;
errorbar(min_num_robots:max_num_robots,avgperconvtime(min_num_robots:max_num_robots),...
    stdperconvtime(min_num_robots:max_num_robots), 'r');
errorbar(min_num_robots:max_num_robots,avgselfconvtime(min_num_robots:max_num_robots),...
    stdselfconvtime(min_num_robots:max_num_robots),'b');
%title('Convergence to the midpoint of Voronoi segment')
legend('Periodic','Self-triggered');
xlabel('Number of robots');
ylabel('Convergence steps');
% 
% figure; hold on;
% plot(min_num_robots:max_num_robots,...
%     avgselfconvtime(min_num_robots:max_num_robots)./avgperconvtime(min_num_robots:max_num_robots));
% title('Ratio of convergence steps')
% xlabel('Number of robots');
% ylabel('Convergence steps for self-triggered/periodic');

figure; hold on;
errorbar(min_num_robots:max_num_robots,avgpercomessage(min_num_robots:max_num_robots),...
    stdpercomessage(min_num_robots:max_num_robots), 'r');
errorbar(min_num_robots:max_num_robots,avgselfcomessage(min_num_robots:max_num_robots),...
    stdselfcomessage(min_num_robots:max_num_robots),'b');
% plot(min_num_robots:max_num_robots,avgpercomessage(min_num_robots:max_num_robots),'r');
% plot(min_num_robots:max_num_robots,avgselfcomessage(min_num_robots:max_num_robots),'b');
%title('Communicaiton among robots')
legend('Periodic','Self-triggered');
xlabel('Number of robots');
ylabel('Average number of communications ');

figure; hold on;
% plot(min_num_robots:max_num_robots,avgpersumswitch(min_num_robots:max_num_robots),'r');
% plot(min_num_robots:max_num_robots,avgselfsumswitch(min_num_robots:max_num_robots),'b');
errorbar(min_num_robots:max_num_robots,avgpersumswitch(min_num_robots:max_num_robots),...
    stdpersumswitch(min_num_robots:max_num_robots),'r');
errorbar(min_num_robots:max_num_robots,avgselfsumswitch(min_num_robots:max_num_robots),...
    stdselfsumswitch(min_num_robots:max_num_robots),'b');
%title('Switchdirection perrobot')
legend('Periodic','Self-triggered');
xlabel('Number of robots');
ylabel('Average direction switching');


figure; hold on;
errorbar(min_num_robots:max_num_robots,avgpersumabsu(min_num_robots:max_num_robots),...
    stdpersumabsu(min_num_robots:max_num_robots),'r');
errorbar(min_num_robots:max_num_robots,avgselfsumabsu(min_num_robots:max_num_robots),...
    stdselfsumabsu(min_num_robots:max_num_robots),'b');
% plot(min_num_robots:max_num_robots,avgpersumabsu(min_num_robots:max_num_robots),'r');
% plot(min_num_robots:max_num_robots,avgselfsumabsu(min_num_robots:max_num_robots),'b');
%title('Average absolute value of control input per robot')
legend('Periodic','Self-triggered');
xlabel('Number of robots');
ylabel('Average absolute value of control input');

figure; hold on;
errorbar(min_num_robots:max_num_robots,avgpersumwongdir(min_num_robots:max_num_robots),...
    stdpersumwongdir(min_num_robots:max_num_robots),'r');
errorbar(min_num_robots:max_num_robots,avgselfsumwongdir(min_num_robots:max_num_robots),...
    stdselfsumwongdir(min_num_robots:max_num_robots),'b');
% plot(min_num_robots:max_num_robots,avgpersumwongdir(min_num_robots:max_num_robots),'r');
% plot(min_num_robots:max_num_robots,avgselfsumwongdir(min_num_robots:max_num_robots),'b');
%title('Average wrong direction perrobot')
legend('Periodic','Self-triggered');
xlabel('Number of robots');
ylabel('Average wrong direction');