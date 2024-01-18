# coding:utf-8
import math
import sys
import os
import pandas as pd
import argparse

# 功能：提取数据集rosbag中的groundtruth位置和姿态，整理成tum格式，RTK全局坐标转换为相对坐标用于精度评估
# 使用：python rtk2tum.py -b [rosbag文件] -o [输出tum格式文件名] -p []topic名称]
# 示例：python bag2gt.py -b Outdoor_09.bag -o stamped_groundtruth.txt -p1 /ublox_driver/receiver_pvt -p2 /mavros/imu/data


def to_xyz_3(M_lat, M_lon, M_alt, O_lat, O_lon, O_alt):
    Ea = 6378137 # 赤道半径
    Eb = 6356752  # 极半径
    M_lat = math.radians(M_lat)
    M_lon = math.radians(M_lon)
    O_lat = math.radians(O_lat)
    O_lon = math.radians(O_lon)
    Ec = Ea * (1 - (Ea - Eb) / Ea * ((math.sin(M_lat)) ** 2)) + M_alt
    Ed = Ec * math.cos(M_lat)
    d_lat = M_lat - O_lat
    d_lon = M_lon - O_lon
    x = d_lat * Ec
   # coding:utf-8
    y = d_lon * Ed
    z = M_alt - O_alt
    return x, y, z

def write_data(path, data):
    with open(path,'a+') as f:
        i = 0
        for item in data:
            # tum格式要求行尾没有空格
            if i != 7:
                f.write(str(item)+" ")
            else:
                 f.write(str(item)+"\n")
            i += 1
        f.close()

# 参数
#setup the argument list
parser = argparse.ArgumentParser(description='turn RTK data into xyz and IMU quternion')
parser.add_argument('-b',  metavar='--bag',  help='rosbag with rtk information')
parser.add_argument('-o', metavar='--output_file',  help='tum file %(default)s')
parser.add_argument('-p1', metavar='--RTK topic name', default="/ublox_driver/receiver_pvt", help='tum file %(default)s')
parser.add_argument('-p2', metavar='--Quternion topic name', default="/mavros/imu/data", help='tum file %(default)s')

args = parser.parse_args()
bagfile = args.b
# 如果不指明输出文件，则与输入文件同名
if args.o is None:
    args.o = args.b.rstrip(".bag")+".tum"
outfile = args.o
#print help if no argument is specified
if len(sys.argv)<2:
    parser.print_help()
    sys.exit(0)
print "bag file is " + bagfile
print "output file is " + outfile
# 删掉输出文件，防止不明追加
os.system("rm -f "+outfile)

# 将rosbag转为临时csv文件
os.system('rostopic echo -b ' + bagfile + ' -p ' + args.p1 +' > temp1.csv')
os.system('rostopic echo -b ' + bagfile + ' -p ' + args.p2 +' > temp2.csv')

# 读取csv临时文件
df1 = pd.read_csv('temp1.csv')
df2 = pd.read_csv('temp2.csv')
# 原点经纬度
O_lat = df1["field.latitude"][0]
O_lon = df1["field.longitude"][0]
O_alt = df1["field.altitude"][0]

for i in range(len(df1)):
    result = []
    time = df1["%time"][i]*1e-9
    result.append(time)
    M_lat = df1["field.latitude"][i]
    M_lon = df1["field.longitude"][i]
    M_alt = df1["field.altitude"][i]
    x, y, z = to_xyz_3(M_lat, M_lon, M_alt, O_lat, O_lon, O_alt)
    qx = df2["field.orientation.x"][5*i]
    qy = df2["field.orientation.y"][5*i]
    qz = df2["field.orientation.z"][5*i]
    qw = df2["field.orientation.w"][5*i]
    result.append(x)
    result.append(y)
    result.append(z)
    result.append(qx)
    result.append(qy)
    result.append(qz)
    result.append(qw)
    write_data(outfile, result)



# 删除临时文件
os.system('rm -f temp1.csv')
os.system('rm -f temp2.csv')

print "Transform finished!"



