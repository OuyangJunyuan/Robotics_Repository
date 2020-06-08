function t = twist_wqh(w,q,h)
if assert_length(w,3,1)==0
    error('w:3x1')
elseif assert_length(h,1,1)==0
    error('h:1x1')
elseif assert_length(q,3,1)==0
    error('q:3x1')
end
if h~=inf
    t=[-cross(w,q)+h*w;w];
else 
    t=[w;[0;0;0]];
end

