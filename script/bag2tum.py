import rosbag
from geometry_msgs.msg import PoseStamped

def convert_bag_to_tum(input_bag, output_file):
    with rosbag.Bag(input_bag, 'r') as bag:
        with open(output_file, 'w') as tum_file:
            for topic, msg, t in bag.read_messages(topics=['/firefly_sbx/vio/odom']): #提取odometry话题名称，以MSCKF-VIO为例
                timestamp = msg.header.stamp.to_nsec()*1e-9
                tx, ty, tz = msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
                qx, qy, qz, qw = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w

                tum_line = "{} {} {} {} {} {} {} {}\n".format(timestamp, tx, ty, tz, qx, qy, qz, qw) #TUM格式
                tum_file.write(tum_line)

if __name__ == "__main__":
    input_bag_file = 'msckf_vio_01.bag'
    output_tum_file = 'msckf.tum'

    convert_bag_to_tum(input_bag_file, output_tum_file)
    print(f"Conversion complete. TUM file saved as {output_tum_file}")

