function [agl,t] = LFPB(aglo,aglf,to,td,dt,a)
if(a< 4*(aglf-aglo)/td^2)
    agl=0;
else
    %参数合理，有直线过渡阶段存在。
    tf=to+td;
    t=to:dt:tf;
    count=1;
    agl=zeros(1,length(t));
    tb=td/2-sqrt((a*td)^2-4*a*(aglf-aglo))/(2*a);
    for tt=t
        if(tt>=to && tt<to+tb)
            agl(count)=aglo+0.5*a*(tt-to)^2;
        elseif(to+tb<=tt && tt<to+td-tb)
            agl(count)=aglo+a*tb*(tt-to-tb/2);
        else 
            agl(count)=aglf-0.5*a*(tf-tt)^2;
        end
        count=count+1;
    end
    t=t;
end

end

