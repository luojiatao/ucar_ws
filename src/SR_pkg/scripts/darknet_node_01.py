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

# #开始在E区域进行视觉识别和统计############################################
#     if data.data=="E1":
#         global darknet_pub
#         global darknet_data
#         if(os.path.exists("/home/luo/ucar_ws/src/sr_pkg/darknet/E1.jpg")):	
#             path='/home/luo/ucar_ws/src/sr_pkg/darknet/E1.jpg'  #图片地址
#             result=my_darknet_SR.darknet(path,net,meta) #识别返回结果
#             darknet_data.E_cuv = result['cuv']
#             darknet_data.E_riv = result['riv']
#             darknet_data.E_whv = result['whv']
#             darknet_data.E_cov = result['cov']
#             print("E1识别情况:"+ str(result))

#     elif data.data=="E2": 
#         if(os.path.exists("/home/luo/ucar_ws/src/sr_pkg/darknet/E2.jpg")):
#             path='/home/luo/ucar_ws/src/sr_pkg/darknet/E2.jpg'        
#             result=my_darknet_SR.darknet(path,net,meta) #识别返回结果1
#             darknet_data.E_cuv =darknet_data.E_cuv + result['cuv']
#             darknet_data.E_riv =  darknet_data.E_riv + result['riv']
#             darknet_data.E_whv = darknet_data.E_whv + result['whv']
#             darknet_data.E_cov =  darknet_data.E_cov + result['cov']
#             print("E2识别情况:"+ str(result))

#     elif data.data=="E3": 
#         if(os.path.exists("/home/luo/ucar_ws/src/sr_pkg/darknet/E3.jpg")):
#             path='/home/luo/ucar_ws/src/sr_pkg/darknet/E3.jpg'   
#             result=my_darknet_SR.darknet(path,net,meta) #识别返回结果1
#             darknet_data.E_cuv =darknet_data.E_cuv + result['cuv']
#             darknet_data.E_riv =  darknet_data.E_riv + result['riv']
#             darknet_data.E_whv = darknet_data.E_whv + result['whv']
#             darknet_data.E_cov =  darknet_data.E_cov + result['cov']
#             print("E3识别情况:"+ str(result))

#     elif data.data=="E4": 
#         if(os.path.exists("/home/luo/ucar_ws/src/sr_pkg/darknet/E4.jpg")):
#             path='/home/luo/ucar_ws/src/sr_pkg/darknet/E4.jpg'   
#             result=my_darknet_SR.darknet(path,net,meta) #识别返回结果1
#             darknet_data.E_cuv =darknet_data.E_cuv + result['cuv']
#             darknet_data.E_riv =  darknet_data.E_riv + result['riv']
#             darknet_data.E_whv = darknet_data.E_whv + result['whv']
#             darknet_data.E_cov =  darknet_data.E_cov + result['cov']
#             print("E4识别情况:"+ str(result))

#             # #发布该房间的识别结果##############################
#             # darknet_pub.publish(darknet_data)

# #开始在D区域进行视觉识别和统计###############################################
#     elif data.data=="D1": 
#         if(os.path.exists("/home/luo/ucar_ws/src/sr_pkg/darknet/D1.jpg")):	
#             path='/home/luo/ucar_ws/src/sr_pkg/darknet/D1.jpg'  #图片地址
#             result=my_darknet_SR.darknet(path,net,meta) #识别返回结果
#             darknet_data.D_cuv = result['cuv']
#             darknet_data.D_riv = result['riv']
#             darknet_data.D_whv = result['whv']
#             darknet_data.D_cov = result['cov']
#             print("D1识别情况:"+ str(result))

#     elif data.data=="D2": 
#         if(os.path.exists("/home/luo/ucar_ws/src/sr_pkg/darknet/D2.jpg")):
#             path='/home/luo/ucar_ws/src/sr_pkg/darknet/D2.jpg'        
#             result=my_darknet_SR.darknet(path,net,meta) #识别返回结果1
#             darknet_data.D_cuv =darknet_data.D_cuv + result['cuv']
#             darknet_data.D_riv =  darknet_data.D_riv + result['riv']
#             darknet_data.D_whv = darknet_data.D_whv + result['whv']
#             darknet_data.D_cov =  darknet_data.D_cov + result['cov']
#             print("D2识别情况:"+ str(result))

#     elif data.data=="D3": 
#         if(os.path.exists("/home/luo/ucar_ws/src/sr_pkg/darknet/D3.jpg")):
#             path='/home/luo/ucar_ws/src/sr_pkg/darknet/D3.jpg'   
#             result=my_darknet_SR.darknet(path,net,meta) #识别返回结果1
#             darknet_data.D_cuv =darknet_data.D_cuv + result['cuv']
#             darknet_data.D_riv =  darknet_data.D_riv + result['riv']
#             darknet_data.D_whv = darknet_data.D_whv + result['whv']
#             darknet_data.D_cov =  darknet_data.D_cov + result['cov']
#             print("D3识别情况:"+ str(result))

#     elif data.data=="D4": 
#         if(os.path.exists("/home/luo/ucar_ws/src/sr_pkg/darknet/D4.jpg")):
#             path='/home/luo/ucar_ws/src/sr_pkg/darknet/D4.jpg'   
#             result=my_darknet_SR.darknet(path,net,meta) #识别返回结果1
#             darknet_data.D_cuv =darknet_data.D_cuv + result['cuv']
#             darknet_data.D_riv =  darknet_data.D_riv + result['riv']
#             darknet_data.D_whv = darknet_data.D_whv + result['whv']
#             darknet_data.D_cov =  darknet_data.D_cov + result['cov']
#             print("D4识别情况:"+ str(result))   

#             #发布识别结果########################################
#             darknet_pub.publish(darknet_data)         
     


            # rospy.signal_shutdown("finish")
            
            
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

