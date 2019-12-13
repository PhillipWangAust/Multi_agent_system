%% 跟随者数
n = 3;

%% 邻接矩阵
A = [ 0  0  3;
      1  0  0
      0  2  0];

%% 领导者
b = [1;
     0;
     0];
B = [1 0 0;
     0 0 0;
     0 0 0];

%% 参数
a = 1;
u = 1.5;
v = 0.5;

%% 滑膜
aa = 1;
bb = 1;
K = 1;  % sint
p = 0.5;
q = 1.5;

%% 初始状态
x_initial = [0;
             1;
             2];
x_leader =   3 ;

gain = 100; % 改变初始状态
x_initial = x_initial*gain;
x_leader = x_leader*gain;

%% 判断是否正定
L = zeros(n);
for i=1:n
    Lii = 0;
    for j=1:n
        if i~=j
            Lii = Lii + A(i,j); 
            L(i,j) = -A(i,j); 
        end
    end
    L(i,i) = Lii;
end

[w,evalue] = eig(L.')
M = zeros(n);
for i=1:n
    M(i,i) = w(i,n); % 左零特征向量可能是第一列，也可能是最后一列！
end
LB = L + B;
eig(LB);           % LB是否可逆？
eig(M*LB+LB.'*M);  % 是否正定？

%% 计算收敛时间
evalues = eig(M*LB+LB.'*M)
evalue_min = evalues(1)  % 最小特征值一定是第一个吗？
w_max = w(2,n)           % 是左特征向量的最大项吗？

L1 = (n/2)*a*evalue_min*power((v+1)/(2*n*w_max), 2*u/(u+1));
L2 = (1/4)*a*evalue_min*power((v+1)/w_max, (u+v)/(u+1));
Tmax = (u+1)/(L1*(u-1)) + (u+1)/(L2*(1-v))