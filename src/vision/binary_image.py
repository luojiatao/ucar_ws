import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError  # 导入缺失的 CvBridgeError
import cv2


class ImageSubscriber:
    def __init__(self, init_node=True):
        if init_node:
            rospy.init_node('image_subscriber', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cam", Image, self.callback)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.process_image(cv_image)
        except CvBridgeError as e:
            print(e)

    def process_image(self, img):
        # 二值化处理
        gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, binary_image = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)  # 处理cv2.threshold返回的两个值
        self.binary_image = binary_image  # 存储二值化图像
        # 显示结果（如果需要）
        cv2.imshow("Binary Image", binary_image)
        cv2.waitKey(3)
        print("Image processed successfully")  # 运行成功时打印的信息

if __name__ == '__main__':
    img_subscriber = ImageSubscriber()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()
