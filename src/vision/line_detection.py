import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from binary_image import ImageSubscriber  # 假设binary_image.py中的类是这样导入的

class LineDetector(ImageSubscriber):
    def process_image(self, img):
        # 首先调用父类的 process_image 来生成并存储二值化图像
        super().process_image(img)
        
        # 接下来进行边缘检测和直线检测
        edges = cv2.Canny(self.binary_image, 50, 150, apertureSize=3)

        # 使用HoughLinesP检测直线
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 100, minLineLength=100, maxLineGap=10)
        

        # 绘制直线
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        # 显示结果
        cv2.imshow("Detected Lines", img)
        cv2.waitKey(3)
        print("Lines detected successfully")

if __name__ == '__main__':
    rospy.init_node('line_detector', anonymous=True)
    detector = LineDetector(init_node=False)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()
