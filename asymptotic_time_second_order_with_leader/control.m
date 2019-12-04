function [sys,x0,str,ts] = control(t,x,u,flag,A,n,r,Kr,Kv) %

switch flag,
    
    case 0, %系统初始化
    [sys,x0,str,ts]=mdlInitializeSizes(n);

    case 1, %连续状态变量的微分函数
    sys=mdlDerivatives(t,x,u);

    case 2, %更新离散状态变量
    sys=mdlUpdate(t,x,u);

    case 3, %系统的输出
    sys=mdlOutputs(t,x,u,A,n,r,Kr,Kv);

    case 4, %变离散采样时间，用于计算下一个采样时刻的绝对时间，ts = [-2 0];
    sys=mdlGetTimeOfNextVarHit(t,x,u);

    case 9, %结束后的处理，清除临时变量等
    sys=mdlTerminate(t,x,u);

    otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
 
end

%系统初始化
function [sys,x0,str,ts]=mdlInitializeSizes(n)

sizes = simsizes;
sizes.NumContStates  = 0; %连续状态的维数
sizes.NumDiscStates  = 0; %离散状态的维数
sizes.NumOutputs     = n; %输出的维数
sizes.NumInputs      = 3*(n+1); %输入的维数
sizes.DirFeedthrough = 1; %输入是否直接影响输出
sizes.NumSampleTimes = 1; %采样时间

sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [0 0];

%连续状态
function sys=mdlDerivatives(t,x,u)

sys = [];

%离散状态
function sys=mdlUpdate(t,x,u)

sys = [];
 
%系统的输出
function sys=mdlOutputs(t,x,u,A,n,r,Kr,Kv) % 邻接矩阵A，跟随者数n，速度比例r

% 控制率
v = u(n+2:2*(n+1));
us = u(2*(n+1)+1:3*(n+1));
dx = zeros(n,1);

for i=1:n
    
    %% 算法1:速度基准：r=0表示无速度交换
%     dx(i) = u(3*(n+1)) - A(i,n+1)*(v(i)-v(n+1));
%     for j=1:n
%         dx(i) = dx(i) - A(i,j)*( (u(i)-u(j)) + r*(v(i)-v(j)) );
%     end
    
    %% 算法2：状态-速度基准：所有航行体可获知基准模型
%     dx(i) = u(3*(n+1)) - A(i,n+1)*( (u(i)-u(n+1)) + r*(v(i)-v(n+1)) );
%     for j=1:n
%         dx(i) = dx(i) - A(i,j)*( (u(i)-u(j)) + r*(v(i)-v(j)) );
%     end
    %% 算法3：状态-速度基准：一般情况
    Ni = 0;
    Ai = 0;
    Bi = 0;
    Ci = 0;
    for j=1:n+1
        Ni = Ni + A(i,j);
        Ai = Ai + A(i,j)* us(j);
        Bi = Bi + A(i,j)* ( u(i)-u(j) );
        Ci = Ci + A(i,j)* ( v(i)-v(j) );
    end
    dx(i) = (1/Ni)*Ai-(1/Ni)*Kr*tanh(Bi)-(1/Ni)*Kv*tanh(Ci);  %有界控制输入
    
end
sys = dx;
 
%变离散采样时间
function sys=mdlGetTimeOfNextVarHit(t,x,u)
 
sys = [];

%结束后处理
function sys=mdlTerminate(t,x,u)
 
sys = [];

