function w_hat = hat(vector36)
if assert_length(vector36,3,1)==1
   w_hat=[0,-vector36(3),vector36(2);vector36(3),0,-vector36(1);-vector36(2),vector36(1),0];
   return;
end
if assert_length(vector36,6,1)==1
   w_hat=[hat(vector36(4:6,1)),vector36(1:3,1);0,0,0,0];
    return;
end
error('vector3:3x1')
end

