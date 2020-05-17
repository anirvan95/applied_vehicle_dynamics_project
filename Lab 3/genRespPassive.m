function X = genRespPassive(sys,u,h)
dsys = c2d(sys, h);
X = [0;0;0;0];
for iter = 1:(length(u)-1)
    zw1 = u(iter);
    zw1_dot = (u(iter+1) - u(iter))/h;
    zw2 = 0;
    zw2_dot = 0;
    X(:,iter+1) = dsys.A*X(:,iter) + dsys.B*[zw1;zw1_dot;zw2;zw2_dot];
end
end

