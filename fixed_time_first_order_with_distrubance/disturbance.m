function [sys,x0,str,ts] = disturbance(t,x,u,flag,n) % 跟随者数

switch flag,
    
    case 0, %系统初始化
    [sys,x0,str,ts]=mdlInitializeSizes(n);

    case 1, %连续状态变量的微分函数
    sys=mdlDerivatives(t,x,u);

    case 2, %更新离散状态变量
    sys=mdlUpdate(t,x,u);

    case 3, %系统的输出
    sys=mdlOutputs(t,x,u,n);

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
sizes.NumInputs      = n; %输入的维数
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
function sys=mdlOutputs(t,x,u,n) % 跟随者数

sys = u;
 
%变离散采样时间
function sys=mdlGetTimeOfNextVarHit(t,x,u)
 
sys = [];

%结束后处理
function sys=mdlTerminate(t,x,u)
 
sys = [];
