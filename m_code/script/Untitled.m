syms a1 a2 a3
a1=0;a2=pi;
zyz=[a1,a2,a3]

e1=[cos(zyz(1)),-sin(zyz(1)),0;...
    sin(zyz(1)),cos(zyz(1)),0;...
    0,0,1]

e2=[cos(zyz(2)),0,-sin(zyz(2));0,1,0;-sin(zyz(2)),0,cos(zyz(2))]
    
e3=[cos(zyz(3)),-sin(zyz(3)),0;...
    sin(zyz(3)),cos(zyz(3)),0;...
    0,0,1]

e1*e2*e3
