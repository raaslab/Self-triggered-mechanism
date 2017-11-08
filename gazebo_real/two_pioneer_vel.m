function two_pioneer_vel(~,~)

global                pioneer0_pub_vel pioneer0_pub_msg
                         %publish target vel_cmd
                         
%                          pioneer0ToGlobal = getTransform(tftree, '/global_origin', '/pioneer_0/base_link'); 
%                          pioneer0Translation=pioneer0ToGlobal.Transform.Translation;
%                          pioneer0x=targetTranslation.X-1;
%                          pioneer0y=targetTranslation.Y;
                         
                         pioneer0_pub_msg.Linear.X=0.3;%1%0.6%0.6%
                         pioneer0_pub_msg.Angular.Z=0.6;%0.6%0.3%
                         %target_pub_msg.Angular.Z=0.05;
                         send(pioneer0_pub_vel,pioneer0_pub_msg);                       
end