min_num_robots  = 3;
max_num_robots = 20;
num_trials = 30;
convthresh = 0.2;

for N = min_num_robots : max_num_robots
    for i = 1  : num_trials
        initheta=round(sort(360*rand(N,1)));
        cla;
        [selfconvtime(i,N),selfcomessage(i,N),selfsumswitch(i,N)] = selffun(N,initheta,convthresh*N);
        %selfconvtime(i,N) = selffun(N,initheta,convthresh*N);
        [perconvtime(i,N),persumswitch(i,N)] = periodicfun(N,initheta,convthresh*N);
    end
end


for N = min_num_robots : max_num_robots
avgperconvtime(N) = mean(perconvtime(:,N));
avgselfconvtime(N) = mean(selfconvtime(:,N));
stdselfconvtime(N) = std(selfconvtime(:,N));
stdperconvtime(N) = std(perconvtime(:,N));

avgpercomessage(N) = 1;
avgselfcomessage(N) = mean(selfcomessage(:,N));

avgpersumswitch(N)=mean(persumswitch(:,N));
avgselfsumswitch(N)=mean(selfsumswitch(:,N));
end

% figure; hold on;
% errorbar(min_num_robots:max_num_robots,avgperconvtime(min_num_robots:max_num_robots),...
%     stdperconvtime(min_num_robots:max_num_robots), 'r');
% errorbar(min_num_robots:max_num_robots,avgselfconvtime(min_num_robots:max_num_robots),...
%     stdselfconvtime(min_num_robots:max_num_robots),'b');
% title('Convergence steps')
% legend('Periodic','Self-triggered');
% xlabel('Number of robots');
% ylabel('Convergence steps');
% 
% figure; hold on;
% plot(min_num_robots:max_num_robots,...
%     avgselfconvtime(min_num_robots:max_num_robots)./avgperconvtime(min_num_robots:max_num_robots));
% title('Ratio of convergence steps')
% xlabel('Number of robots');
% ylabel('Convergence steps for self-triggered/periodic');

% figure; hold on;
% plot(min_num_robots:max_num_robots,avgpercomessage(min_num_robots:max_num_robots),'r');
% plot(min_num_robots:max_num_robots,avgselfcomessage(min_num_robots:max_num_robots),'b');
% legend('Periodic','Self-triggered');
% xlabel('Number of robots');
% ylabel('Commessage perrobot');

figure; hold on;
plot(min_num_robots:max_num_robots,avgpersumswitch(min_num_robots:max_num_robots),'r');
plot(min_num_robots:max_num_robots,avgselfsumswitch(min_num_robots:max_num_robots),'b');
legend('Periodic','Self-triggered');
xlabel('Number of robots');
ylabel('Switchdirection perrobot');