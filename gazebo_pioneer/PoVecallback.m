function [robot0_pose]=PoVecallback(~,message,pub,msg, goal)
% Implementation of proportional position control
% For comparison to Simulink implementation
% Tunable parameters
%wgain = 5; % Gain for the angular velocity [rad/s / rad]
%vconst = .5; % Linear velocity when far away [m/s]
%distThresh = 0.5; % Distance treshold [m]
% Generate a simplified pose
pos = message.Position;
ori = message.Orientation;
% From quaternion to Euler
%angles = quat2eul([quat.W quat.X quat.Y quat.Z]);
%theta = angles(1);
robot0_pose = [pos.X, pos.Y, ori.W];  % X, Y, Theta
[roll, pitch, yaw] = quat2angle([ori.W ori.X ori.Y ori.Z],'XYZ');
%yaw
% Proportional Controller
% v = 0; % default linear velocity
% w = 0; % default angluar velocity
% distance = sqrt((pose(1)-goal(1))^2+(pose(2)-goal(2))^2);
% if (distance > distThresh)
%     v = vconst;
%     desireYaw = atan2(goal(2)-pose(2),goal(1)-pose(1));
%     u = desireYaw-theta;
%     bound = atan2(sin(u),cos(u));
%     w = min(0.5 , max(-0.5, wgain*bound));
% end
% Publish
%cmdmsg.Header.stamp=now;

msg.Linear.X=goal(1);
msg.Linear.Y=0;
msg.Linear.Z=0;
msg.Angular.X=0;
msg.Angular.Y=0;
msg.Angular.Z=goal(2);
send(pub,msg);
pause(2)

%display(pose)
%fprintf('OdomCallback_v1: x=%4.1f,y=%4.1f dist=%4.2f, cmd.v=%4.2f, cmd.w=%4.2f\n', ...
    %pose(1),pose(2),distance, v, w);
  %fprintf(pose(1),pose(1),);
end