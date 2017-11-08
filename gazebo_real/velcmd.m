function [linear_vel, angle_vel]=velcmd(robot_vd_angle, robot_yaw, u_angle_abs)
  global k_wtov;
  global angle_kp;
  
    if(robot_vd_angle-robot_yaw)>=-2*pi && (robot_vd_angle-robot_yaw)<-3*pi/2
     % velocity +, angle_diff +
       linear_vel=k_wtov*u_angle_abs;
       angle_vel=angle_kp*(robot_vd_angle+2*pi-robot_yaw);
    elseif (robot_vd_angle-robot_yaw)>=-3*pi/2 && (robot_vd_angle-robot_yaw)<-pi/2
      % velocity -
       linear_vel=-k_wtov*u_angle_abs;
       angle_vel=angle_kp*(robot_vd_angle+pi-robot_yaw);
    elseif (robot_vd_angle-robot_yaw)>=-pi/2 && (robot_vd_angle-robot_yaw)<pi/2
       % velocity +
       linear_vel=k_wtov*u_angle_abs;
       angle_vel=angle_kp*(robot_vd_angle-robot_yaw);
    elseif (robot_vd_angle-robot_yaw)>=pi/2 && (robot_vd_angle-robot_yaw)<3*pi/2
       % velocity -
       linear_vel=-k_wtov*u_angle_abs;
       angle_vel=angle_kp*(robot_vd_angle-robot_yaw-pi); 
    else % 3*pi/2~2*pi
       % velocity +
       linear_vel=k_wtov*u_angle_abs;
       angle_vel=angle_kp*(robot_vd_angle-robot_yaw-2*pi); 
    end
    
end