function g = gab_Rp(R,p)
if assert_length(R,3,3) && assert_length(p,3,1)
    g=[R,p;0,0,0,1];
else
    error('R:3x3,p:3x1');
end

