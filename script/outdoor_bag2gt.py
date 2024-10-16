# coding:utf-8
import math
import sys
import os
import pandas as pd
import argparse

# Function: Extract groundtruth position and orientation from the rosbag dataset, organize it into TUM format,
# convert RTK global coordinates to relative coordinates for accuracy evaluation
# Usage: python rtk2tum.py -b [rosbag file] -o [output TUM format filename] -p []topic name]
# Example: python bag2gt.py -b Outdoor_09.bag -o stamped_groundtruth.txt -p1 /ublox_driver/receiver_pvt -p2 /mavros/imu/data

def to_xyz_3(M_lat, M_lon, M_alt, O_lat, O_lon, O_alt):
    Ea = 6378137  # Equatorial radius
    Eb = 6356752  # Polar radius
    M_lat = math.radians(M_lat)
    M_lon = math.radians(M_lon)
    O_lat = math.radians(O_lat)
    O_lon = math.radians(O_lon)
    Ec = Ea * (1 - (Ea - Eb) / Ea * ((math.sin(M_lat)) ** 2)) + M_alt
    Ed = Ec * math.cos(M_lat)
    d_lat = M_lat - O_lat
    d_lon = M_lon - O_lon
    x = d_lat * Ec
    y = d_lon * Ed
    z = M_alt - O_alt
    return x, y, z

def write_data(path, data):
    with open(path,'a+') as f:
        i = 0
        for item in data:
            # The TUM format requires no spaces at the end of each line
            if i != 7:
                f.write(str(item) + " ")
            else:
                f.write(str(item) + "\n")
            i += 1
        f.close()

# Argument setup
parser = argparse.ArgumentParser(description='turn RTK data into xyz and IMU quaternion')
parser.add_argument('-b',  metavar='--bag',  help='rosbag with RTK information')
parser.add_argument('-o', metavar='--output_file',  help='TUM file %(default)s')
parser.add_argument('-p1', metavar='--RTK topic name', default="/ublox_driver/receiver_pvt", help='TUM file %(default)s')
parser.add_argument('-p2', metavar='--Quaternion topic name', default="/mavros/imu/data", help='TUM file %(default)s')

args = parser.parse_args()
bagfile = args.b
# If output file is not specified, use the same name as the input file
if args.o is None:
    args.o = args.b.rstrip(".bag") + ".tum"
outfile = args.o
# Print help if no argument is specified
if len(sys.argv) < 2:
    parser.print_help()
    sys.exit(0)
print("Bag file is " + bagfile)
print("Output file is " + outfile)
os.system("rm -f " + outfile)

# Convert rosbag to temporary CSV file
os.system('rostopic echo -b ' + bagfile + ' -p ' + args.p1 + ' > temp1.csv')
os.system('rostopic echo -b ' + bagfile + ' -p ' + args.p2 + ' > temp2.csv')

# Read the temporary CSV files
df1 = pd.read_csv('temp1.csv')
df2 = pd.read_csv('temp2.csv')
# Origin latitude, longitude, and altitude
O_lat = df1["field.latitude"][0]
O_lon = df1["field.longitude"][0]
O_alt = df1["field.altitude"][0]

for i in range(len(df1)):
    result = []
    time = df1["%time"][i] * 1e-9
    result.append(time)
    M_lat = df1["field.latitude"][i]
    M_lon = df1["field.longitude"][i]
    M_alt = df1["field.altitude"][i]
    x, y, z = to_xyz_3(M_lat, M_lon, M_alt, O_lat, O_lon, O_alt)
    qx = df2["field.orientation.x"][5 * i]
    qy = df2["field.orientation.y"][5 * i]
    qz = df2["field.orientation.z"][5 * i]
    qw = df2["field.orientation.w"][5 * i]
    result.append(x)
    result.append(y)
    result.append(z)
    result.append(qx)
    result.append(qy)
    result.append(qz)
    result.append(qw)
    write_data(outfile, result)

os.system('rm -f temp1.csv')
os.system('rm -f temp2.csv')

print("Transform finished!")
