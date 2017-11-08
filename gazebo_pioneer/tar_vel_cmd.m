function tar_vel_cmd()

global                target_pub_vel target_pub_msg
                         %publish target vel_cmd
                         target_pub_msg.Linear.X=1;%1%0.6%0.6%   0.3
                         target_pub_msg.Angular.Z=0.6;%0.6%0.3%   0.6
                         %target_pub_msg.Angular.Z=0.05;
                         send(target_pub_vel,target_pub_msg);                       
end