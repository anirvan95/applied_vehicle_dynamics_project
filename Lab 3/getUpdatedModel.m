function [Ash, Bsh, Ask, Bsk] = getUpdatedModel(m,j,k,L)
k1 = k;
k2 = k;
l1 = L;
l2 = L;

Ash = [0, 1, 0, 0;
    -(k1+k2)/m, 0, (k1*l1 - k2*l2)/m, 0;
    0, 0, 0, 1;
    (k1*l1 - k2*l2)/j, 0, -(k1*l1*l1 + k2*l2*l2)/j, 0];
Bsh = [0,0,0,0;
    k1/m, k2/m, -1/m, -1/m;
    0,0,0,0;
    -k1*l1/j, k2*l2/j, l1/j, -l2/j];

Ask=[0 1 0 0
    -2*k/m 0 0 0
    0 0 0 1
    0 0 -2*k*L^2/j 0];
Bsk=[0 0 0 0
    k/m k/m -1/m -1/m
    0 0 0 0
    -L*k/j L*k/j L/j -L/j];