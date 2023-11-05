后续测试：
1.电机数据收发（主要为关节电机）
    实现效果：通电后拨动电机，反馈角度值解算后 正常
2.机体数据更新
状态向量：
    tehta->关节电机角度、phi
    tehta_dot ->δtheta

    x   里程计，x_dot累加
    x_dot  速度，rpm转换得出，与驱动轮电机有关
    phi     直接测量（pitch轴角度）
    phi_dot  直接测量
    当前航向角   yaw角速度
    L0  左右轮腿长，计算得到，由机械参数+电机角度
    r   横滚角   直接可得（roll）

3.解算

系统：参考哈工程的系统图
输入为期望的状态向量Xd，
控制律  u=K（Xd-X1）        X1为当前状态，K为填入的反馈增益矩阵
T_l=T-pd（δ航向）   T_r=T+pd（δ航向）    
δ航向=当前航向-期望航向

关节电机部分解算：
保持平衡，因此
Tp_l=Tp-(theta_r-theta_l)
Tp_r=Tp+(theta_r-theta_l)
theta ：摆杆与竖直方向夹角，theta=phi_0-phi
phi为机体与水平夹角（具体为imu的pitch角度）
phi_0为5连杆中的，由电机角度phi_1和phi_4解算得出。
f_l=pid(Ld_l-L0_l)；f_r=pid(Ld_r-L0_r);
r_d为期望横滚角，r_0为当前横滚角（roll）
F_l = f_l+Kr(r_d-r_0)
F_r = f_l-Kr(r_d-r_0)
vmc部分：(分左右)
        输入  F_x、Tp_x
        输出  T1_x,T2_x
