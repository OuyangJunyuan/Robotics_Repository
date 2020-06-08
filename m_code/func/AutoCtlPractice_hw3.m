ctl_num=[1,1.0];ctl_den=[1,1];
obj_num=[1,1];obj_den=myconv([1,-1],[1,3]);

ctl=tf(ctl_num,ctl_den);
obj=tf(obj_num,obj_den);
sys=feedback(obj,ctl);
[A,B,C,D]=tf2ss([1 3],[1 4])

impulse(sys);
