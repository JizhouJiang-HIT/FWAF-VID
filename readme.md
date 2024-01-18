# FlapVI: Visual-Inertial Datasets for Flapping-Wing Aerial Vehicles  Towards Fast Maneuvering with Benchmark Evaluation  

The detailed introduction of the FlapVI dataset can be found in our paper: “**FlapVI: Visual-Inertial Datasets for Flapping-Wing Aerial Vehicles  Towards Fast Maneuvering with Benchmark Evaluation  **” by Jizhou Jiang, Erzhen Pan, Wenfu Xu, Wei Sun.

## Video of FlapVI



## The Platform ：HIT-Hawk

![image-20240117205516246](C:\Users\admin\AppData\Roaming\Typora\typora-user-images\image-20240117205516246.png)

The platform and sensor configuration of the eagle-like large-scale flapping-wing aerial vehicle **HIT-Hawk**, with a wingspan of 1.80 meters and body weight of 770 grams, it can continuously autonomous flight for a maximum of 180 minutes while carrying payload equivalent to 100% of its weight, with a takeoff weight of 1550 grams.

## ROS Topic (Synchronized)

|      | Description                                                  | Topic                         | Type                            |
| ---- | :----------------------------------------------------------- | :---------------------------- | ------------------------------- |
| 1    | Realsense D435 depth image                                   | /camera/depth/image_rect_raw  | sensor_msgs/Image               |
| 2    | Realsense D435 left gray image                               | /camera/infra1/image_rect_raw | sensor_msgs/Image               |
| 3    | Realsense D435 right gray image                              | /camera/infra2/image_rect_raw | sensor_msgs/Image               |
|      | Realsense D435 color image                                   | /camera/color/image_raw       | sensor_msgs/Image               |
| 4    | The build-in IMU of D435i                                    | /camera/imu                   | sensor_msgs/Imu                 |
| 5    | The filltered fusion IMU data from CUAV V5+                  | /mavros/imu/data              | sensor_msgs/Imu                 |
| 6    | Ephemeris data for GLONASS (GLO) satellites                  | /ublox_driver/glo_ephem       | gnss_comm/GnssGloEphemMsg       |
| 7    | Range measurement data of RTK                                | /ublox_driver/range_meas      | gnss_comm/GnssMeasMsg           |
| 8    | The latitude, longitude, and altitude coordinates of the receiver (LLA stands for Longitude, Latitude, and Altitude) | /ublox_driver/receiver_lla    | sensor_msgs/NavSatFix           |
| 9    | The Position, Velocity, and Time (PVT) data of the RTK receiver | /ublox_driver/receiver_pvt    | gnss_comm/GnssPVTSolnMsg        |
| 10   | The data related to time pulse information in the u-blox driver | /ublox_driver/time_pulse_info | gnss_comm/GnssTimePulseInfoMsg  |
| 11   | Linktrack P UWB positioning tag data（trasn. and rota.）     | /nlink_linktrack_tagframe0    | nlink_parser/LinktrackTagframe0 |



## Dataset Sequences list

We provide the option to download the raw dataset files and the corresponding extracted motion trajectory files, both saved in rosbag format. The download links for each sequence and trajectory are attached in the table. If the links in the table cannot be directly opened, please copy the corresponding cloud storage link and paste it into your browser for downloading. 

If neither of these methods works, we also provide links to the cloud storage containing all the datasets.

| #    | sequences (link)                                 |    camera perspective     | illumination |         trajectory(link)         | length(m) | duration(s) | max ground speed(m/s) | air speed(m/s) | groundtruth | level     |
| ---- | ------------------------------------------------ | :-----------------------: | :----------: | :------------------------------: | :-------: | :---------: | :-------------------: | :------------: | :---------: | --------- |
| 1    | [Outdoor 1](https://pan.quark.cn/s/a176ecdb9dcd) | 45 degree downward facing | bright scene |               oval               |  910.18   |     156     |         12.02         |      0.36      |      ✔      | medium    |
| 2    | Outdoor 2                                        | 45 degree downward facing | bright scene |               oval               |  839.24   |     150     |         13.40         |      0.39      |      ✔      | medium    |
| 3    | Outdoor 3                                        |      Forward  facing      | bright scene |               oval               |  1150.60  |     197     |         13.47         |      1.59      |      ✔      | difficult |
| 4    | Outdoor 4                                        | 45 degree downward facing | bright scene |           8-character            |  808.28   |     159     |         10.04         |      1.68      |      ✔      | medium    |
| 5    | Outdoor 5                                        | 45 degree downward facing | bright scene |           8-character            |  963.88   |     155     |         11.74         |      1.53      |      ✔      | medium    |
| 6    | Outdoor 6                                        |      Forward  facing      | bright scene |           8-character            |  575.89   |     125     |         11.09         |      1.75      |      ✔      | difficult |
| 7    | Outdoor 7                                        | 45 degree downward facing | bright scene | Fast Flapping and Gliding Switch |  377.71   |     67      |         12.29         |      1.62      |      ✔      | medium    |
| 8    | Outdoor 8                                        |      Forward  facing      | bright scene | Fast Flapping and Gliding Switch |  104.10   |     41      |         9.87          |      1.64      |      ✔      | medium    |
| 9    | Outdoor 9                                        | 45 degree downward facing | dark  scene  |              random              |   78.15   |     26      |         8.34          |      1.34      |      ✔      | difficult |
| 10   | Outdoor 10                                       | 45 degree downward facing | dark  scene  |              random              |  105.86   |    31.5     |         7.87          |      1.47      |      ✔      | difficult |
| 11   | Outdoor 11                                       |      Forward  facing      | dark  scene  |              random              |   94.31   |    34.4     |         8.92          |      1.44      |      ✔      | difficult |
| 12   | Outdoor 12                                       |      Forward  facing      | dark  scene  |              random              |   98.82   |     69      |         9.27          |      1.51      |      ✔      | difficult |
| 13   | Indoor 13                                        | 45 degree downward facing | bright scene |               oval               |   43.25   |    17.6     |         8.94          |      NaN       |      ✔      | medium    |
| 14   | Indoor 14                                        | 45 degree downward facing | bright scene |               oval               |  163.44   |    36.3     |         12.17         |      NaN       |      ✔      | difficult |
| 15   | Indoor 15                                        |      Forward  facing      | bright scene |               oval               |   83.34   |    29.3     |         8.81          |      NaN       |      ✔      | difficult |
| 16   | Indoor 16                                        | 45 degree downward facing | dark  scene  |              random              |   55.28   |    13.4     |         11.85         |      NaN       |      ✔      | difficult |
| 17   | Indoor 17                                        |      Forward  facing      | dark  scene  |              random              |   51.14   |    14.6     |         11.99         |      NaN       |      ✔      | difficult |



#### 



