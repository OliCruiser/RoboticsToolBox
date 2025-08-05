import numpy as np
from spatialmath import SO3


def zyx_rotation_matrix(alpha_deg, beta_deg, gamma_deg):
    """
    根据 ZYX 欧拉角（度）计算旋转矩阵 R_ab。
    参数：
        alpha_deg: 绕 Z 轴旋转角度（度）
        beta_deg: 绕 Y 轴旋转角度（度）
        gamma_deg: 绕 X 轴旋转角度（度）
    返回：
        R: 3x3 旋转矩阵
    """
    # 转换为弧度
    alpha = np.deg2rad(alpha_deg)
    beta = np.deg2rad(beta_deg)
    gamma = np.deg2rad(gamma_deg)

    # 使用 spatialmath 计算 ZYX 旋转矩阵
    R = SO3.Rz(alpha) * SO3.Ry(beta) * SO3.Rx(gamma)

    return R


def main():
    # 用户输入欧拉角（度）
    try:
        alpha_deg = 10
        beta_deg = 20
        gamma_deg = 30

        # 计算旋转矩阵
        R = zyx_rotation_matrix(alpha_deg, beta_deg, gamma_deg)

        # 打印结果，保留 4 位小数
        print("\nZYX 欧拉角对应的旋转矩阵 R_ab：")
        print(np.round(R, 4))

    except ValueError:
        print("错误：请输入有效的数字！")


if __name__ == "__main__":
    main()