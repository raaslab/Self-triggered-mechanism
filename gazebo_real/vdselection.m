function [i_vd]=vdselection(i_edge_flag, linear_vel)

global edge;

if i_edge_flag==edge(1,1)
    if sign(linear_vel)<0
        i_vd=edge(1,2:3);
    elseif sign(linear_vel)>0
        i_vd=edge(1,4:5);
    else 
        i_vd=[0 0];
    end
    
elseif i_edge_flag==edge(2,1)
    if sign(linear_vel)<0
        i_vd=edge(2,2:3);
    elseif sign(linear_vel)>0
        i_vd=edge(2,4:5);
    else 
        i_vd=[0 0];
    end
elseif i_edge_flag==edge(3,1)
    if sign(linear_vel)<0
        i_vd=edge(3,2:3);
    elseif sign(linear_vel)>0
        i_vd=edge(3,4:5);
    else 
        i_vd=[0 0];
    end
else
    if sign(linear_vel)<0
        i_vd=edge(4,2:3);
    elseif sign(linear_vel)>0
        i_vd=edge(4,4:5);
    else
        i_vd=[0 0];
    end

end