function [Q, A]= cal_Q(t, path)
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
T = 1; % define the T;
Q = eye(5);
n = size(path, 3);
t_step = 0.05; 

for i = 0:4
    for j = 0:4
        Q(i + 1 , j + 1) = i*(i - 1)*(i - 2) *(i - 3) * j * (j - 1)* (j - 2) * (j - 3) /(i + j - 7 ) * T^(i + j -7);
    end
end

A_c1 = zeros(1, 5);
for i = 0:4
    A_c1(1, i+1) = T^(i - 0);
end

A_c2 = zeros(4, 10);
for i = 1:4
    for j = 0:4
        A_c2(i, j+1) =  ;
    end
end






end

