function [x] = ppt_mothed(N)
dt=0.01;%�������;
t1=0:dt:dt*N;%������Ӧ������
K=500;T=0.025;alphaT=0.15;a=0.5;N=length(t1)-1;%����ֵ
et=5;%��������ͷ������ʱ��
num=K*[T,1];den=[alphaT,1,0];Gk=tf(num,den);  e=feedback(1,Gk);e=series(e,tf(1,[1,0])); %���ݺ���
%%%%%%%%%%%%%%%%%%%%%%
w=impulse(e,t1);
%%%%%%%%%%%%%%%%%%%%%
t2=-et-dt*N:dt:et;%����ʱ���
A=atan(a*t2);
u=a*cos(A).^2; %�����ź�
%%%%
for k=1:length(t2)-N
    x(k)=0;
    for n=k-N:k
        x(k)=x(k)+ w(1+k-n)*u(n+N); %�ӵ�N����ʼ���������ǰN�����У��õ�����N+k�ľ��ֵ
    end
end
x=dt*x;
end

