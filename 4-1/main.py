'''
写出一个计算 4.4 节中三连杆操作臂的逆运动学子程序。这个子程序应以如下形式传递变量。

Procedure INVKIN(VAR wrelb: frame; VAR current, near, far: vec3; VAR sol: boolean);

其中，wrelb是一个输入量，代表相对于基坐标系的腕部坐标系；current也是一个输入量，代表机器人当前的位置信息（通过关节角矢量给出）；near是最接近的解；far是第二个解；sol是一个表示是否找到解的标志（如果未找到解，则Sol = FALSE）。连杆长度（米）为
l 1 =l 2 =0.5

关节运动范围为
−170 ∘ ≤θ i ≤170 ∘ 

测试你的程序，用KIN反复调用这个程序，证明它们的确是互逆的。
'''
import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3

# ---------------------------
# 1. 定义机械臂（3R平面）
# ---------------------------
l1 = 0.5
l2 = 0.5

arm = DHRobot([
    RevoluteDH(a=0, alpha=0, d=0),      # θ1
    RevoluteDH(a=l1, alpha=0, d=0),     # θ2
    RevoluteDH(a=l2, alpha=0, d=0)      # θ3
], name="3R Planar Arm")

# ---------------------------
# 2. 逆运动学子程序
# ---------------------------
def INVKIN(wrelb: SE3, current: np.ndarray):
    """
    三连杆机械臂逆运动学
    wrelb   : SE3，腕部相对基坐标系的位姿
    current : np.array([θ1, θ2, θ3]) 当前关节角(rad)
    返回: near, far, sol
    """
    # 末端位置
    px, py = wrelb.t[0], wrelb.t[1] # 齐次变换矩阵里有

    # 距离
    r = np.sqrt(px**2 + py**2)

    # 解是否存在
    if r > l1 + l2 or r < abs(l1 - l2):
        return None, None, False

    # ---------------- 计算 θ2 ----------------
    cos_theta2 = (px**2 + py**2 - l1**2 - l2**2) / (2 * l1 * l2)
    theta2_1 = np.arccos(np.clip(cos_theta2, -1, 1))   # 解1 np.clip 变量 cos_theta2 的值限制在区间 [-1, 1] 
    theta2_2 = -theta2_1                               # 解2

    # ---------------- 计算 θ1 ----------------
    # calc_theta1 是 INVKIN 的局部函数，只能在 INVKIN 内部被调用。
    def calc_theta1(theta2):
        k1 = l1 + l2 * np.cos(theta2)
        k2 = l2 * np.sin(theta2)
        return np.arctan2(py, px) - np.arctan2(k2, k1)

    theta1_1 = calc_theta1(theta2_1)
    theta1_2 = calc_theta1(theta2_2)

    # ---------------- 计算 θ3 ----------------
    phi = np.arctan2(wrelb.R[1,0], wrelb.R[0,0])  # 末端方向角
    theta3_1 = phi - (theta1_1 + theta2_1)
    theta3_2 = phi - (theta1_2 + theta2_2)

    # ---------------- 两组解 ----------------
    sol1 = np.array([theta1_1, theta2_1, theta3_1])
    sol2 = np.array([theta1_2, theta2_2, theta3_2])

    # ---------------- 选near和far ----------------
    dist1 = np.linalg.norm(sol1 - current) # 计算 sol1 和 current 两个关节角解之间的欧氏距离（范数）
    dist2 = np.linalg.norm(sol2 - current)

    if dist1 < dist2:
        near, far = sol1, sol2
    else:
        near, far = sol2, sol1

    return near, far, True


# ---------------------------
# 3. 测试
# ---------------------------
if __name__ == "__main__":
   
    # 给定一个关节角
    q = np.deg2rad([30, 45, 60])
    T = arm.fkine(q)   # 正运动学

    print("正运动学结果：")
    print(T) # 齐次变换矩阵

    near, far, sol = INVKIN(T, q)

    if sol:
        print("近解 (deg):", np.rad2deg(near))
        print("远解 (deg):", np.rad2deg(far))
    else:
        print("没有解！")
