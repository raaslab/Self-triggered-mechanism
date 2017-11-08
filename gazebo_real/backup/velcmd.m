function [linear_vel, angle_vel]=velcmd(robot_vd_angle, robot0_pose_last)
  global k_wtov;
  global angle_kp;
  linear_vel_constant=0.28;
    if(robot_vd_angle-robot0_pose_last(3))>=-2*pi && (robot_vd_angle-robot0_pose_last(3))<-3*pi/2
     % velocity +, angle_diff +
       linear_vel=linear_vel_constant;
       angle_vel=angle_kp*(robot_vd_angle+2*pi-robot0_pose_last(3));
    elseif (robot_vd_angle-robot0_pose_last(3))>=-3*pi/2 && (robot_vd_angle-robot0_pose_last(3))<-pi/2
      % velocity -
       linear_vel=-linear_vel_constant;
       angle_vel=angle_kp*(robot_vd_angle+pi-robot0_pose_last(3));
    elseif (robot_vd_angle-robot0_pose_last(3))>=-pi/2 && (robot_vd_angle-robot0_pose_last(3))<pi/2
       % velocity +
       linear_vel=linear_vel_constant;
       angle_vel=angle_kp*(robot_vd_angle-robot0_pose_last(3));
    elseif (robot_vd_angle-robot0_pose_last(3))>=pi/2 && (robot_vd_angle-robot0_pose_last(3))<3*pi/2
       % velocity -
       linear_vel=-linear_vel_constant;
       angle_vel=angle_kp*(robot_vd_angle-robot0_pose_last(3)-pi); 
    else % 3*pi/2~2*pi
       % velocity +
       linear_vel=linear_vel_constant;
       angle_vel=angle_kp*(robot_vd_angle-robot0_pose_last(3)-2*pi); 
    end
    
end