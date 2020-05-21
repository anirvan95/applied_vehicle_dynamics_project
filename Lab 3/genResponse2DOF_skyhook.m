function y = genResponse2DOF_skyhook(u,h,T)
mp = 0.16;
ms = 0.16;
cp = 0.8;
kp = 6.32;
ks = 0.0632;

Ask = [0, 1, 0, 0;-ks/ms, -T/ms, ks/ms, 0;0, 0, 0, 1;ks/mp, T/mp, -(ks + kp)/mp, -cp/mp];
Bsk = [0, 0; 0, 0; 0, 0; kp/mp, cp/mp];
Csk = [1, 0, 0, 0];
Dsk = [0,0];
sys = ss(Ask,Bsk,Csk,Dsk);

dsys = c2d(sys, h);
y = [0;0;0;0];
for iter = 1:(length(u)-1)
    zw = u(iter);
    zw_dot = (u(iter+1) - u(iter))/h;
    y(:,iter+1) = dsys.A*y(:,iter) + dsys.B*[zw;zw_dot];
end
end
