global  T
global  pioneer0_pub_vel pioneer0_pub_msg

%waitForTransform(tftree, '/global_origin', '/pioneer_0/base_link');

T=0.5;

pioneer0_pub_vel = rospublisher('/pioneer_0/RosAria/cmd_vel');
pioneer0_pub_msg = rosmessage('geometry_msgs/Twist');


t = timer('TimerFcn',{@two_pioneer_vel},'Period',T,'ExecutionMode','fixedSpacing');
start(t);