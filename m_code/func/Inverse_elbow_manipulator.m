function gst= Inverse_elbow_manipulator(l,angle,gst0)
q1=[0;0;0];q2=[0;0;l(5)];q3=[0;0;l(4)];q4=[0;l(3);l(4)];q5=[0;l(2)+l(3);l(4)];q6=[0;l(2)+l(3)+l(1);l(4)];
w1=[0;0;1];w2=[0;1;0];w3=[-1;0;0];w4=[-1;0;0];w5=[-1;0;0];w6=[0;0;1];
g1=twist2gab(twist_wqh(w1,q1,0),angle(1));
g2=twist2gab(twist_wqh(w2,q2,0),angle(2));
g3=twist2gab(twist_wqh(w3,q3,0),angle(3));
g4=twist2gab(twist_wqh(w4,q4,0),angle(4));
g5=twist2gab(twist_wqh(w5,q5,0),angle(5));
g6=twist2gab(twist_wqh(w6,q6,0),angle(6));
gst=g6*g5*g4*g3*g2*g1*gst0;
end

