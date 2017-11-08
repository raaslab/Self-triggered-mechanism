%goal =[0,0];  % Goal position in x/y/z
% Setup publisher
cmdpub = rospublisher('/robot_0/move_base_simple/goal');
cmdmsg = rosmessage('geometry_msgs/PoseStamped');
pause(2);
goal = [2,2];
% Setup subscription - which implemets our controller.
% We pass the publisher, the message to publish and the goal as
% additional parameters to the callback function.

odomsub = rossubscriber('/robot_0/pose',{@OdomCallback_v1,cmdpub,cmdmsg, goal});

% % give value to the cmdmsg
% % 
% %cmdmsg.Header.stamp=now;
% cmdmsg.Header.FrameId='map';
% cmdmsg.Pose.Position.X=2.5;
% cmdmsg.Pose.Position.Y=10;
% cmdmsg.Pose.Position.Z=0;
% cmdmsg.Pose.Orientation.W=1.0;
% 
% %odomsub = rossubscriber('/robot_0/move_base_simple/goal', @exampleHelperROSChatterCallback);
% %Publish a message to the /robot_0/move_base_simple/goal topic.
% send(cmdpub,cmdmsg);
% % pause(2);
