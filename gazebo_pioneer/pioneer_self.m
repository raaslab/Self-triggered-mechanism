cmdpub = rospublisher('/robot_0/cmd_vel');
cmdmsg = rosmessage('geometry_msgs/Twist');
pause(2);
goal = [0.0,0.0];
% Setup subscription - which implemets our controller.
% We pass the publisher, the message to publish and the goal as
% additional parameters to the callback function.

robot_0 = rossubscriber('/robot_0/pose',{@PoVecallback,cmdpub,cmdmsg, goal});
%robot_1 = rossubscriber('/robot_1/pose',{@PoVecallback,cmdpub,cmdmsg, goal});


% cmdmsg.Linear.X=goal(1);
% cmdmsg.Angular.Z=goal(2);
% send(cmdpub,cmdmsg); 
% pause(2);



% cmdmsg.Linear.X=goal(1);
% cmdmsg.Linear.Y=goal(2);
% cmdmsg.Linear.Z=0;
% cmdmsg.Angular.Z=goal(3);
% send(cmdpub,cmdmsg);
% pause(2);

% for i = 1 : 10
%     i
%     pause(1);
% end