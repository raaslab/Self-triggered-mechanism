function robot0_stop(robot_0_pub_vel,robot_0_pub_msg)

  rota_cmd = [0.0,0.0];
  robot_0_pub_msg.Linear.X=rota_cmd(1);
  robot_0_pub_msg.Angular.Z=rota_cmd(2); 
  %[robot_0_pub_msg.Linear.X, robot_0_pub_msg.Angular.Z]=rota_cmd;
  send(robot_0_pub_vel,robot_0_pub_msg);  
  
end