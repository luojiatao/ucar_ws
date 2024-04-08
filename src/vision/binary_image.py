import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

# 初始化CvBridge
bridge = CvBridge()

def imgmsg_to_cv2(data):
    """
    将ROS图像消息转换为OpenCV图像。
    """
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        return cv_image
    except CvBridgeError as e:
        print(e)
        return None

def process_image(img):
    """
    对收到的图像进行二值化处理，并显示结果。
    """
    gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, binary_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)
    # 显示结果
    resized_image = cv2.resize(binary_image, (640, 480))  # 可以调整这里的尺寸以适应显示需求
    cv2.imshow("Binary Image", resized_image)
    cv2.waitKey(3)
    print("Image processed successfully")

def image_callback(data):
    """
    图像订阅的回调函数，负责接收图像消息，并进行处理。
    """
    cv_image = imgmsg_to_cv2(data)
    if cv_image is not None:
        process_image(cv_image)

def setup_subscriber():
    """
    设置ROS节点和图像订阅者。
    """
    rospy.init_node('image_subscriber', anonymous=True)
    rospy.Subscriber("/cam", Image, image_callback)

def main():
    """
    主函数,启动ROS节点和图像订阅。
    """
    setup_subscriber()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
