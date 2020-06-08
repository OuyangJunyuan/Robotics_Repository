function g = twist2gab(xi,theta)
if assert_length(xi,6,1) && assert_length(theta,1,1)
    v=xi(1:3,1);
    w=xi(4:6,1);
    if w==0 
        g=[eye(3,3),v*theta;0,0,0,1];
    else
        R=Rort(w,theta);
        g=[R,(eye(3,3)-R)*cross(w,v)+w*w'*v*theta;0,0,0,1];
    end
else
    error('xi:6x1,theta:1x1');
end
end

