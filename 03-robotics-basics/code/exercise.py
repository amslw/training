"""基础作业完成前两个函数，进阶作业完成第三个函数。距离 m，角度 rad。"""
import math
from robotics_demo import rotate2d


def optical_to_link(point_optical):
    """理想共原点安装：光学 X右Y下Z前 → link X前Y左Z上。"""
    # 基础 1：根据前、左、上对应的光学分量，返回三个坐标。
    x_optical, y_optical, z_optical = point_optical
    link_x = z_optical
    link_y = -x_optical
    link_z = -y_optical
    return (link_x, link_y, link_z)
    


def aim_angles(point_reference):
    """输入固定瞄准参考系中的三维点，返回 (yaw, elevation)，均为 rad。

    参考系 X前Y左Z上，理想转轴共点。elevation 向上为正。
    如果输入有非有限值，或水平距离 <= 1e-12 m，应抛出 ValueError。
    """
    # 基础 2：使用 isfinite、hypot 和 atan2，实现上述约定。
    x, y, z = point_reference
    if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
        raise ValueError("输入包含非有限值")
    horizontal_distance = math.hypot(x, y)
    if horizontal_distance <= 1e-12:
        raise ValueError("水平距离过小")
    yaw = math.atan2(y, x)
    elevation = math.atan2(z, horizontal_distance)
    return (yaw, elevation)
    


def small_yaw_to_base(point_small, big_yaw, small_yaw, small_offset, big_offset):
    """将小 yaw 中的三维点换到底盘中，返回 (x, y, z)。

    两根 yaw 轴均竖直，角度均为相对父坐标系的角度，向左为正。
    small_offset：小 yaw 原点在大 yaw 平台 L 中的位置，用 L 的轴表达。
    big_offset：大 yaw 原点在底盘 B 中的位置，用 B 的轴表达。
    p_L = Rz(small_yaw) p_S + small_offset
    p_B = Rz(big_yaw) p_L + big_offset
    """
    # 进阶：调用两次 rotate2d，分别得到 L、B 中的水平分量。
    # 每次旋转后加该段偏移，Z 只累加对应的高度偏移。
    x_s, y_s, z_s = point_small
    x_rotated_L, y_rotated_L = rotate2d(x_s, y_s, small_yaw)
    x_L = x_rotated_L + small_offset[0]
    y_L = y_rotated_L + small_offset[1]
    z_L = z_s + small_offset[2]
    x_B, y_B = rotate2d(x_L, y_L, big_yaw)
    x_B = x_B + big_offset[0]
    y_B = y_B + big_offset[1]
    z_B = z_L + big_offset[2]
    return (x_B, y_B, z_B)
    
