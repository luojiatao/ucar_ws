import cv2
import numpy as np

class LineFollower:
    def __init__(self, video_file):
        self.video_capture = cv2.VideoCapture(video_file)
        self.lower_white = np.array([0, 0, 200])
        self.upper_white = np.array([180, 25, 255])

    def create_white_mask(self, hsv):
        white_mask = cv2.inRange(hsv, self.lower_white, self.upper_white)
        return white_mask

    def find_target_point(self, binary_image):
        # 初始化左右最近的点
        leftmost = None
        rightmost = None

        # 获取图像的尺寸
        height, width = binary_image.shape

        # 从左向右遍历
        for x in range(width):
            for y in range(height):
                if binary_image[y, x] != 0:
                    leftmost = (x, y)
                    break
            if leftmost is not None:
                break

        # 从右向左遍历
        for x in range(width - 1, -1, -1):
            for y in range(height):
                if binary_image[y, x] != 0:
                    rightmost = (x, y)
                    break
            if rightmost is not None:
                break

        # 如果找到了左右最近的点，计算中点
        if leftmost is not None and rightmost is not None:
            target_point = ((leftmost[0] + rightmost[0]) // 2, (leftmost[1] + rightmost[1]) // 2)
        else:
            # 如果没有找到，返回图像中心点
            target_point = (width // 2, height // 2)

        return target_point

    def steering_control(self, x, width):
        # 假设x坐标与转向命令成线性关系
        max_steering_angle = 1.0  # 假设这是最大转向角度
        steering_angle = (x / width - 0.5) * max_steering_angle
        return steering_angle

    def publish_cmd(self, cmd):
        # 打印命令
        print(f"cmd: {cmd}")

    def process_frame(self):
        ret, frame = self.video_capture.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            return

        # 将BGR图像转换为HSV色彩空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        white_mask = self.create_white_mask(hsv)

        # 使用掩码提取白色线条，并转换回BGR空间
        white_lines = cv2.bitwise_and(frame, frame, mask=white_mask)

        # 对白色目标进行二值化处理
        gray = cv2.cvtColor(white_lines, cv2.COLOR_BGR2GRAY)
        _, binary_image = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY)

        # 寻找目标点
        target_point = self.find_target_point(binary_image)

        # 计算转向命令
        cmd = self.steering_control(target_point[0], frame.shape[1])

        # 打印命令
        self.publish_cmd(cmd)

        # 在图像上显示目标点
        cv2.circle(frame, target_point, 5, (0, 0, 255), -1)
        cv2.imshow('Frame', frame)

        # 按下 'q' 键退出循环
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return

if __name__ == '__main__':
    video_file = r'C:\Users\luojiatao\Desktop\ucar_ws\src\vision\001.mp4'  # 替换为你的视频文件路径
    line_follower = LineFollower(video_file)

    while True:
        line_follower.process_frame()
