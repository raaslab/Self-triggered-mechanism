function robot0_move(robot0_pose_last,V,next_boundary_orientation,robot_0_pub_vel,robot_0_pub_msg)
  epsilon_cornor=0.5;
  %crosspoint
  if sqrt((robot0_pose_last(1)-V(1,1))^2+(robot0_pose_last(2)-V(2,1))^{2})<=epsilon_cornor;
    
    if robot0_pose_last(3)==next_boundary_orientation
       rota_cmd = [0.5,0.0];
       robot_0_pub_msg.Linear.X=rota_cmd(1);
       robot_0_pub_msg.Angular.Z=rota_cmd(2); 
       %[robot_0_pub_msg.Linear.X, robot_0_pub_msg.Angular.Z]=rota_cmd;
       send(robot_0_pub_vel,robot_0_pub_msg); 
        
    else
       rota_cmd = [0.0,0.1];
       robot_0_pub_msg.Linear.X=rota_cmd(1);
       robot_0_pub_msg.Angular.Z=rota_cmd(2); 
       %[robot_0_pub_msg.Linear.X, robot_0_pub_msg.Angular.Z]=rota_cmd;
       send(robot_0_pub_vel,robot_0_pub_msg); 
              
    end
          
  else
  rota_cmd = [0.5,0.0];
  robot_0_pub_msg.Linear.X=rota_cmd(1);
  robot_0_pub_msg.Angular.Z=rota_cmd(2); 
  %[robot_0_pub_msg.Linear.X, robot_0_pub_msg.Angular.Z]=rota_cmd;
  send(robot_0_pub_vel,robot_0_pub_msg); 
    
  end
  
end