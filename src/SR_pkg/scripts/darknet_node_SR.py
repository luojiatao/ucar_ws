#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy,os
from std_msgs.msg import String
from std_msgs.msg import Int8
from darknet_msgs.msg import darknet
# import my_darknet_SR
import my_darknet_SR
# import match
import sys


def recognize_and_count(img_path, area):
    result = my_darknet_SR.darknet(img_path, net, meta)
    if area == "E":
        darknet_data.E_cuv += result["cuv"]
        darknet_data.E_riv += result["riv"]
        darknet_data.E_whv += result["whv"]
        darknet_data.E_cov += result["cov"]
    elif area == "D":
        darknet_data.D_cuv += result["cuv"]
        darknet_data.D_riv += result["riv"]
        darknet_data.D_whv += result["whv"] 
        darknet_data.D_cov += result["cov"]
        # ...其他区域

def callback(data):

    if data.data == "begin_yolo": 
        print("已接收begin_yolo信号")

         # 获得所有图片路径
        for img_path in os.listdir("/home/luo/ucar_ws/src/sr_pkg/darknet"):
            img_paths.append("/home/luo/ucar_ws/src/sr_pkg/darknet/" + img_path)
        
        # 识别和统计所有图片
        for img_path in img_paths:
            try:
                area = img_path.split("/")[-1][0]
                recognize_and_count(img_path, area)
            except:
                print("出现错误,跳过图片:{}".format(img_path))
    
        # 发布统计结果
        darknet_pub.publish(darknet_data)
        print("完成视觉统计,发布darknet_data数据!")
        rospy.signal_shutdown("finish")

    else:
        print("等待begin_yolo信号!")

            
            
if __name__ == '__main__':

    rospy.init_node('darknet_node', anonymous=False)
    darknet_pub=rospy.Publisher("/darknet_result",darknet,queue_size=1)
    darknet_data=darknet()
    global net
    global meta
    global darknet_data
    img_paths = []
    print("开始加载darknet yolov3神经网络!")
    net,meta =my_darknet_SR.load_net_for_ready()
    print("数据载入成功!  darknet yolov3神经网络成功接入!")
    rospy.Subscriber("/goal_arrive", String, callback)


    try:
        rospy.spin()     
    except rospy.ROSInterruptException:
        rospy.loginfo("text node terminated.")

