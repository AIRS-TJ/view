# view
安装依赖evo：

$ pip install evo --upgrade --no-binary evo

运行步骤:

1.对齐两条轨迹

$ ./align.py data/1/fuse1811.bag /vrpn_client_node/bluerov/pose /odometry/filtered

在当前目录下输出output.bag

2.截取图像的主题

$ rosbag filter data/1/2021-08-27-18-11all.bag output2.bag "topic=='/davis_left/image_raw'"

在当前目录下输出output2.bag

3.合并包，将output.bag和output2.bag合并成output3.bag

$ ./merge_bag.py -v output3.bag output.bag output2.bag

在当前目录下输出output3.bag

4.显示

$ roslaunch view demo.launch 
