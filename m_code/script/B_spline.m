%%本代码实现Trajectory Planning for Automatic Machines and Robots Springer 书中的算法
clear all;
dt = 0.001;

P=[0,0; 1,0; 1,1; 2,1; 2,0; 3,0; 3,1];
p = 3;

n = length(P)-1;
[knot,m] = gen_knot(n,p,2)

plot(P(:,1),P(:,2),'-r');hold on;
axis([0,3,0,3]);

uset = knot(1):dt:knot(end)-0.01;
for u = uset
    i = WhichSpan(knot,u,m,p); %m = m+1 -1  ,得到 u ∈ [u_i,u_i+1]
    N = BasisFunc(knot,u,i,p);
    
    pr=[0,0];
    for d=1:2
        for j=1:p+1 
            pr(d) = pr(d) +  P(i - p + j,d) *N(j);
        end
    end
    plot(pr(1),pr(2),'.r');
end
%
function [knot_table,m] = gen_knot(n,p,type)
%m：节点数m+1个。[u0,...,um]
%n：控制点个数。 [P0,...,Pn]
%p：曲线次数。   p
%type：曲线种   类. 1：open 2：clampped 3：closed
%默认均匀分布
m = n + p + 1;
switch type
    case 1
        knot_table = (0:1/m:1);
    case 2
        temp = m - 2 * p ;
        knot_table = (0:1/temp:1);
        knot_table = [0*ones(1,p),knot_table];
        knot_table = [knot_table,1*ones(1,p)];
    case 3
end
end

%% BasisFunc 求得N_{i-p+0}^{p}(u) ~ N_{i-p+p}^{p}(u) 其余为0.
function N = BasisFunc(U,u,i,p)

    temp = 0; acc = 0;  
    DR = []; DL = [];
    N = []; %从左边开始算起
    N(1) = 1;
    for j = 1:p %i j 都按从0开始的，下面的idx +1 是因为matlab idx从1开始
        DL(j +1) = u - U(i+1-j +1);
        DR(j +1) = U(i+j +1) - u;
        acc = 0;
        for r=0:j-1
            temp = N(r +1) / (DR(r+1 +1)+DL(j-r +1));
            N(r +1) = acc + DR(r+1 +1)*temp;
            acc = DL(j-r +1)*temp;
        end
        N(j +1) = acc;
    end
end

%% WhichSpan:二分法查找u所在细分节点区间
function i = WhichSpan(U,u,m,p)
    high = m + 1 - p;
    low = p + 1 ;
    mid = 0;
    if u == U(high)
        mid = high ;
    else
        mid = fix(( high + low )/2); %fix 函数截取整数
        while u < U(mid) || u >= U(mid+1) %当 mid<=u && u<=mid+1 时 idx = mid, i = mid -1
            if u == U(mid+1)
               mid = mid + 1;
            else
                if u > U(mid)
                    low = mid;
                else 
                    high = mid ;
                end
                mid = fix(( high + low )/2);
            end
        end
    end 
    i = mid - 1;
end