function[boundary_id]=check_boundary(x,y,P,Q,R,S)

    epsilon_boundary=2;
    %(x2-x1)*(y-y1)=(y2-y1)*(x-x1)
    if abs((Q(1,1)-P(1,1))*(y-P(2,1))-(Q(2,1)-P(2,1))*(x-P(1,1)))<=epsilon_boundary
       boundary_id=1;

    elseif abs((R(1,1)-Q(1,1))*(y-Q(2,1))-(R(2,1)-Q(2,1))*(x-Q(1,1)))<=epsilon_boundary
       boundary_id=2;

    elseif abs((S(1,1)-R(1,1))*(y-R(2,1))-(S(2,1)-R(2,1))*(x-R(1,1)))<=epsilon_boundary
       boundary_id=3; 

    elseif abs((P(1,1)-S(1,1))*(y-S(2,1))-(P(2,1)-S(2,1))*(x-S(1,1)))<=epsilon_boundary
       boundary_id=4;
    
    else
       boundary_id=0;  

    end


end