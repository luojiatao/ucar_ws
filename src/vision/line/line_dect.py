import cv2
import numpy
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

#在02基础上增加形态学操作保证曲线的连续性

class LineFollower(object):
    def __init__(self):
        rospy.init_node('line_follower', anonymous=True)
        self.cv_bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/cam', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist = Twist()

    def image_callback(self, msg):
        try:
            # 转换图像格式
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        # 调整图像大小
        cv_image = cv2.resize(cv_image, (320, 240))
        
        # 转换到HSV颜色空间
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)


        # 白色的HSV范围
        lower_white = numpy.array([0, 0, 200])
        upper_white = numpy.array([180, 25, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)


      # 应用形态学操作
        kernel = numpy.ones((5, 5), numpy.uint8)

        # 使用闭运算连接断开的线条片段
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)

        # 使用膨胀操作增强线条
        white_mask = cv2.dilate(white_mask, kernel, iterations=1)

        # 提取白色线条
        white_lines = cv2.bitwise_and(cv_image, cv_image, mask=white_mask)

        # 计算重心
        M = cv2.moments(white_lines)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(cv_image, (cx, cy), 10, (0, 0, 255), -1)

            # 计算中心误差
            h, w, d = cv_image.shape
            error = cx - w / 2

            # 根据线条位置调整转向
            if cv2.countNonZero(white_mask) == 1:
                if cx < w / 2:
                    self.twist.angular.z = -0.05  # 右转
                else:
                    self.twist.angular.z = 0.05  # 左转
            else:
                self.twist.angular.z = -float(error) * 0.005

            self.twist.linear.x = 0.2 if abs(error) < 50 else 0.1
            self.cmd_vel_pub.publish(self.twist)

        # 显示结果
        cv2.imshow("Window", cv_image)
        cv2.imshow("White Lines", white_lines)
        cv2.waitKey(3)

def main():
    try:
        line_follower = LineFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
