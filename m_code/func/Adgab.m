function adg = Adgab(g)
if assert_length(g,4,4)==1
    R=g(1:3,1:3);
    p=g(1:3,4);
    adg=[R,hat(p)*R;zeros(3,3),R];
else
    error('g:4x4');
end
end

