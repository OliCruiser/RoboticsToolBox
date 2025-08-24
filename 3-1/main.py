import roboticstoolbox as rtb
import numpy as np
from math import radians

# 定义3R机器人L1=4 L2=3 L3=2
robot=rtb.DHRobot(
    [
        rtb.RevoluteDH(a=4,d=0,alpha=0,qlim=[-np.pi, np.pi],name='J1'),
        rtb.RevoluteDH(a=3,d=0,alpha=0,qlim=[-np.pi, np.pi],name='J2'),
        rtb.RevoluteDH(a=2,d=0,alpha=0,qlim=[-np.pi, np.pi],name='J3'),
    ],
    name='3R Robot'
)
# 查看机器人信息
print("机器人关节信息:")
print(robot)
# 正运动学计算
# 定义关节角
q_list=[
    [0, 0, 0],
    [10,20,30],
    [90,90,90]
]
for i,q_deg in enumerate(q_list):
    q_rad=np.radians(q_deg)  # 转换为弧度
    T=robot.fkine(q_rad)  # 正运动学计算
    print(f"关节角 {q_deg}° 的末端执行器位姿:")
    print(T)