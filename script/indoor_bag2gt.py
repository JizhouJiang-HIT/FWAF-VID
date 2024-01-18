# coding:utf-8
import sys
import os
import pandas as pd
import argparse
import numpy as np
from scipy.optimize import least_squares

#功能：提取数据集rosbag中的uwb groundtruth位置与姿态，进行TOA解算后保存为TUM格式

def write_data(path, data):
    with open(path, 'a+') as f:
        i = 0
        for item in data:
            if i != 7:
                f.write(str(item) + " ")
            else:
                f.write(str(item) + "\n")
            i += 1
        f.close()

def toa_residuals(params, anchor_positions, distances):
    x, y = params
    return np.sqrt((x - anchor_positions[:, 0])**2 + (y - anchor_positions[:, 1])**2) - distances

def fit_toa(anchor_positions, distances):
    initial_guess = [0, 0]  # Initial guess for x, y
    result = least_squares(toa_residuals, initial_guess, args=(anchor_positions, distances))
    return result.x

def convert_to_tum_with_toa(bagfile, outfile, topic):
    # 调用 rostopic echo 导出 CSV 文件
    os.system('rostopic echo -b ' + bagfile + ' -p ' + topic + ' > temp.csv')

    # 读取 CSV 文件
    df = pd.read_csv('temp.csv')

    # 进行 TOA 定位
    anchor_positions = np.array([[0, 0], [1, 0], [0, 1], [1, 1]])  # 替换为实际的基站位置

    # 将 TOA 定位结果转为 TUM 格式并写入输出文件
    for i in range(len(df)):
        # 更新距离信息
        distances = np.array([df["field.dis_arr0"][i], df["field.dis_arr1"][i], df["field.dis_arr2"][i], df["field.dis_arr3"][i]])
        
        if len(distances) == 4:
            # 进行 TOA 定位
            estimated_position = fit_toa(anchor_positions, distances)

            # 更新位置
            pos_2d = estimated_position

            # 写入 TUM 文件
            result = [
                df["%time"][i] * 1e-9,
                pos_2d[0],  # x-coordinate
                pos_2d[1],  # y-coordinate
                df["field.pos_3d2"][i],  # z-coordinate from field.pos3d2
                df["field.quaternion0"][i],
                df["field.quaternion1"][i],
                df["field.quaternion2"][i],
                df["field.quaternion3"][i]
            ]
            write_data(outfile, result)

    # 删除临时文件
    os.remove('temp.csv')

if __name__ == "__main__":
    # 设置命令行参数解析器
    parser = argparse.ArgumentParser(description='Convert rosbag to TUM format with TOA fitting')
    parser.add_argument('-b', metavar='--bag', help='rosbag with uwb information')
    parser.add_argument('-o', metavar='--output_file', help='TUM file')
    parser.add_argument('-p', metavar='--topic_name', default="/nlink_linktrack_tagframe0", help='topic name')

    # 解析命令行参数
    args = parser.parse_args()

    # 如果没有指定输出文件，则与输入文件同名
    if args.o is None:
        args.o = args.b.rstrip(".bag") + "_toa_fit.tum"

    # 执行转换
    convert_to_tum_with_toa(args.b, args.o, args.p)

    print(f"Conversion finished. TUM format data with TOA fitting saved to {args.o}")

