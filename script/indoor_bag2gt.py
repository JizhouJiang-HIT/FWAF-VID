# coding:utf-8
import sys
import os
import pandas as pd
import argparse
import numpy as np
from scipy.optimize import least_squares

# Function: Extract UWB groundtruth positions and orientations from the rosbag dataset,
# solve using TOA, and save the results in TUM format

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
    return np.sqrt((x - anchor_positions[:, 0]) ** 2 + (y - anchor_positions[:, 1]) ** 2) - distances


def fit_toa(anchor_positions, distances):
    initial_guess = [0, 0]  # Initial guess for x, y
    result = least_squares(toa_residuals, initial_guess, args=(anchor_positions, distances))
    return result.x


def convert_to_tum_with_toa(bagfile, outfile, topic):
    # Use rostopic echo to export CSV file
    os.system('rostopic echo -b ' + bagfile + ' -p ' + topic + ' > temp.csv')

    # Read the CSV file
    df = pd.read_csv('temp.csv')

    # Perform TOA positioning
    anchor_positions = np.array([[0, 0], [1, 0], [0, 1], [1, 1]])  # Replace with actual anchor positions

    # Convert TOA results to TUM format and write to output file
    for i in range(len(df)):
        # Update distance information
        distances = np.array(
            [df["field.dis_arr0"][i], df["field.dis_arr1"][i], df["field.dis_arr2"][i], df["field.dis_arr3"][i]])

        if len(distances) == 4:
            # Perform TOA positioning
            estimated_position = fit_toa(anchor_positions, distances)

            # Update the position
            pos_2d = estimated_position

            # Write to TUM file
            result = [
                df["%time"][i] * 1e-9,
                pos_2d[0],  # x-coordinate
                pos_2d[1],  # y-coordinate
                df["field.pos_3d2"][i],  # z-coordinate from field.pos_3d2
                df["field.quaternion0"][i],
                df["field.quaternion1"][i],
                df["field.quaternion2"][i],
                df["field.quaternion3"][i]
            ]
            write_data(outfile, result)
    os.remove('temp.csv')


if __name__ == "__main__":
    # Set up command line argument parser
    parser = argparse.ArgumentParser(description='Convert rosbag to TUM format with TOA fitting')
    parser.add_argument('-b', metavar='--bag', help='rosbag with UWB information')
    parser.add_argument('-o', metavar='--output_file', help='TUM file')
    parser.add_argument('-p', metavar='--topic_name', default="/nlink_linktrack_tagframe0", help='topic name')

    args = parser.parse_args()

    if args.o is None:
        args.o = args.b.rstrip(".bag") + "_toa_fit.tum"

    convert_to_tum_with_toa(args.b, args.o, args.p)

    print(f"Conversion finished. TUM format data with TOA fitting saved to {args.o}")
