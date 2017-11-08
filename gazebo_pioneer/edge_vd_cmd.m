function  [linear_vel, angle_vel]=edge_vd_cmd(robot_pose_last,i,u_angle_abs,u_angle_sign)
    global edge vertex cornor_epsilon vd i_edge_flag
    
    
        % if the robot is close to the left vertex of the edge
if sqrt((robot_pose_last(i,1)-edge(i_edge_flag(i),1))^2+(robot_pose_last(i,2)-edge(i_edge_flag(i),2))^2)<=cornor_epsilon
        % enter the neighborhood, first stop
        %robot_pub_msg.Linear.X=0;
        % if the velocity -, the edge -1;
        if u_angle_sign<0
            %select the previous edge
            i_edge_flag(i)=i_edge_flag(i)-1;
            if i_edge_flag(i)<=0;
                i_edge_flag(i)=4;
            end
            % select the left vertex of the previous edge
            vd(i,1:2)=edge(i_edge_flag(i),1:2);
            roboti_vd_angle=atan2pi((vd(i,2)-robot_pose_last(i,2)),(vd(i,1)-robot_pose_last(i,1)));
            [linear_vel, angle_vel]=velcmd(roboti_vd_angle, robot_pose_last(i,3),u_angle_abs);
        else
            % stay the current edge,
            % i_edge_flag(i)=i_edge_flag(i);
            % select the right vertex of the current edge
            vd(i,1:2)=edge(i_edge_flag(i),3:4);
            roboti_vd_angle=atan2pi((vd(i,2)-robot_pose_last(i,2)),(vd(i,1)-robot_pose_last(i,1)));
            [linear_vel, angle_vel]=velcmd(roboti_vd_angle,robot_pose_last(i,3),u_angle_abs);
        end
        
        
        % if the robot is close to the right vetex of the edge
    elseif sqrt((robot_pose_last(i,1)-edge(i_edge_flag(i),3))^2+(robot_pose_last(i,2)-edge(i_edge_flag(i),4))^2)<=cornor_epsilon
        % enter the neighborhood, first stop
        %robot_pub_msg.Linear.X=0;
        % if the velocity +, the edge +1;
        if u_angle_sign>0
            %select the next edge
            i_edge_flag(i)=i_edge_flag(i)+1;
            if i_edge_flag(i)>length(vertex);
                i_edge_flag(i)=1;
            end
            % select the right vertex of the next edge
            vd(i,1:2)=edge(i_edge_flag(i),3:4);
            roboti_vd_angle=atan2pi((vd(i,2)-robot_pose_last(i,2)),(vd(i,1)-robot_pose_last(i,1))); 
            [linear_vel, angle_vel]=velcmd(roboti_vd_angle,  robot_pose_last(i,3),u_angle_abs);
        else
            % stay the current edge,
            % i_edge_flag(i)=i_edge_flag(i);
            % select the left vertex of the current edge
            vd(i,1:2)=edge(i_edge_flag(i),1:2);
            roboti_vd_angle=atan2pi((vd(i,2)-robot_pose_last(i,2)),(vd(i,1)-robot_pose_last(i,1)));
            [linear_vel, angle_vel]=velcmd(roboti_vd_angle,robot_pose_last(i,3),u_angle_abs);
        end
        
        % esle the robot is between the left & right vertex of the edge
    else
        
        if u_angle_sign<0
            vd(i,1:2)=edge(i_edge_flag(i),1:2);
            roboti_vd_angle=atan2pi((vd(i,2)-robot_pose_last(i,2)),(vd(i,1)-robot_pose_last(i,1)));
            [linear_vel, angle_vel]=velcmd(roboti_vd_angle, robot_pose_last(i,3),u_angle_abs);
        else
            vd(i,1:2)=edge(i_edge_flag(i),3:4);
            roboti_vd_angle=atan2pi((vd(i,2)-robot_pose_last(i,2)),(vd(i,1)-robot_pose_last(i,1)));
            [linear_vel, angle_vel]=velcmd(roboti_vd_angle, robot_pose_last(i,3),u_angle_abs);
        end


end