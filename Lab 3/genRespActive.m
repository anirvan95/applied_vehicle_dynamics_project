function [X, F] = genRespActive(sys,u,h,cx,cz)
l=6;
dsys = c2d(sys, h);
X = [0;0;0;0];
F = [0;0];
for iter = 1:(length(u)-1)
    zw1 = u(iter);
    zw2 = 0;
    Fa1 = -(cx*X(4,iter) - cz*l*X(2,iter))/(2*l);
    Fa2 = (cx*X(4,iter) + cz*l*X(2,iter))/(2*l);
    F(:,iter+1) = [Fa1;Fa2];
    X(:,iter+1) = dsys.A*X(:,iter) + dsys.B*[zw1;zw2;Fa1;Fa2];
end
end
