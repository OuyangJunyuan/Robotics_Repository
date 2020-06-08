function R = Rort(w,theta)
if assert_length(w,3,1) && assert_length(theta,1,1)
    ma=norm(w);
    w=w/ma;
    theta=theta*ma;
    w_hat=[0,-w(3),w(2);w(3),0 ,-w(1); -w(2),w(1) ,0];
    R=eye(3,3)+w_hat*sin(theta)+w_hat^2*(1-cos(theta));
else
    error('input1:1x3£¬input2£º1x1');
end
end

