function [sys,x0,str,ts] = control1(t,x,u,flag,A,b,n,a,uu,v) % 邻接矩阵，领导信息，跟随者数

switch flag,
    
    case 0, %系统初始化
    [sys,x0,str,ts]=mdlInitializeSizes(n);

    case 1, %连续状态变量的微分函数
    sys=mdlDerivatives(t,x,u);

    case 2, %更新离散状态变量
    sys=mdlUpdate(t,x,u);

    case 3, %系统的输出
    sys=mdlOutputs(t,x,u,A,b,n,a,uu,v);

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
sizes.NumInputs      = n+1; %输入的维数
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
function sys=mdlOutputs(t,x,u,A,b,n,a,uu,v)

%控制率
dx = zeros(n,1);
for i=1:n
    for j=1:n
        dx(i) = dx(i) + A(i,j)*(u(i)-u(j)) + b(i)*(u(i)-u(n+1));
    end
    dx(i) = -a*sig(dx(i),uu)-a*sig(dx(i),v);
end
sys = dx;
 
%变离散采样时间
function sys=mdlGetTimeOfNextVarHit(t,x,u)
 
sys = [];

%结束后处理
function sys=mdlTerminate(t,x,u)
 
sys = [];

