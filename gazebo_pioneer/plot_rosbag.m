%function positions = plot_rosbag(filepath,posetopics,animate)

positions = [];

  animate = true;
  posetopics{1} = '/robot_0/p3dx/base_pose_ground_truth';
   posetopics{2} = '/robot_1/p3dx/base_pose_ground_truth';
   posetopics{3} = '/robot_2/p3dx/base_pose_ground_truth';
   posetopics{4} = '/robot_3/p3dx/base_pose_ground_truth';
   posetopics{5} = '/robot_4/p3dx/base_pose_ground_truth';
% 
   posetopics{6} = '/robot_5/RosAria/pose';
   posetopics{7} = '/robot_6/RosAria/pose';



% Read in the bag file
bagselect = rosbag('/home/tulipa/2017-10-31-22-48-56.bag');

% Select and plot each topic
% for i = 1 : length(posetopics), 'Time', [0 10],
%     topicselect(i) = select(bagselect,  'Topic',posetopics{i});
for i = 1 : length(posetopics)
    topicselect(i) = select(bagselect,'Topic',posetopics{i});
    allMsgs = readMessages(topicselect(i));
    
    positions{i} = [];
    for j = 1 : length(allMsgs)
        positions{i} = [positions{i}; ...
            allMsgs{j}.Pose.Pose.Position.X, ...
            allMsgs{j}.Pose.Pose.Position.Y, ...
            allMsgs{j}.Pose.Pose.Position.Z];
    end

end

% % Plot things
% figure(1); axis equal; box on; hold on;
% 
% axis([-11 11 -11 11]);

%%
if animate == false
    for i = 1 : length(positions)
            if i<=5
            plot(positions{i}(1,1), positions{i}(1,2), 'bo', 'MarkerSize',10)
            plot(positions{i}(end,1), positions{i}(end,2), 'rd', 'MarkerSize',10)
            plot(positions{i}(:,1), positions{i}(:,2), '-', 'LineWidth', 1)
            elseif i == 6
            plot(positions{i}(1,1)-2.43, positions{i}(1,2)+2.72, 'bo', 'MarkerSize',10)
            plot(positions{i}(end,1)-2.43, positions{i}(end,2)+2.72, 'rd', 'MarkerSize',10)
            plot(positions{i}(:,1)-2.43, positions{i}(:,2)+2.72, '-', 'LineWidth', 1)
            elseif i== 7     
            plot(positions{i}(:,1), positions{i}(:,2), '-', 'LineWidth', 1)
            plot(positions{i}(1,1), positions{i}(1,2), 'bo', 'MarkerSize',10)
            plot(positions{i}(end,1), positions{i}(end,2), 'rd', 'MarkerSize',10)
            end
        %plot(positions{i}(:,1), positions{i}(:,2), '-o', 'LineWidth', 2)
        %text(positions{i}(end,1), positions{i}(end,2),sprintf('R%d',i-1),'FontSize',14);
       
    end
    
else
    j = 1;
    notdone = ones(length(posetopics),1);
    while sum(notdone) > 0
        % Plot things
        figure(1); axis equal; box on; hold on;

        axis([-11 11 -11 11]);
        title('The trajectories of six robots and one target','fontsize',12)
        xlabel({'$$x$$'},'Interpreter','latex','fontsize',11)
        ylabel({'$$y$$'},'Interpreter','latex','fontsize',11)
         cla; hold on;
        for i = 1 : length(positions)
            if (j > length(positions{i}(:,1)))
                notdone(i) = 0;
                continue;
            end
            if i<=5
            plot(positions{i}(1:j*10,1), positions{i}(1:j*10,2), '-o', 'LineWidth', 1)
            elseif i == 6
            plot(positions{i}(1:j,1)-2.43, positions{i}(1:j,2)+2.72, '-o', 'LineWidth', 1)
            elseif i== 7     
            plot(positions{i}(1:j,1), positions{i}(1:j,2), '-*', 'LineWidth', 0.5)
            end
               
            %text(positions{i}(end,1), positions{i}(end,2),sprintf('R%d',i-1),'FontSize',14);
        end
        legend('r1','r2','r3','r4','r5','r6','target','fontsize',24)

        j = j + 1;
        pause(0.1);
    end
end