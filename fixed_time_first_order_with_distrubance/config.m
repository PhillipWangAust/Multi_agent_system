% 智能体数
n = 3;

% 邻接矩阵
A =  [ 0  1  0;
       1  0  2
       0  2  0];

% 参数
a = 1;
b = 1;
r = 2;
c = 1; % c>=d,抗干扰项

% 初始状态
x_initial = [0;
             1;
             2];

% 计算收敛时间
u = 1 - 1/r;
v = 1 + 1/r;

Au = A.^(1/2-1/(4*r));
Av = A.^(1/2+1/(4*r));
Lu = zeros(n);
Lv = zeros(n);
for i=1:n
    Luii = 0;
    Lvii = 0;
    for j=1:n
        if i~=j
            Luii = Luii + Au(i,j); 
            Lvii = Lvii + Av(i,j); 
            Lu(i,j) = -Au(i,j); 
            Lv(i,j) = -Av(i,j); 
        end
    end
    Lu(i,i) = Luii;
    Lv(i,i) = Lvii;
end

evalues_u = eig(Lu);
evalues_v = eig(Lv);
evalue2_u = evalues_u(2);  % 最小正特征值一定是第二个吗？
evalue2_v = evalues_v(2);

Tmax1 = 2/(a*power(2,u)*power(evalue2_u,(u+1)/2)*(1-u)) + 2/(b*power(2,v)*power(power(n,2),(1-v)/2)*power(evalue2_v,(v+1)/2)*(v-1))
Tmax2 = pi*r*power(power(n,2),1/(4*r))/(2*power(a*b,1/2)*power(evalue2_u,1/2-1/(4*r))*power(evalue2_v,1/2+1/(4*r)))

