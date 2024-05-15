import cv2
import numpy as np

class LineFollower:
    def __init__(self, video_file):
        # 初始化视频捕获对象
        self.video_capture = cv2.VideoCapture(video_file)
        # 创建用于形态学操作的核
        self.kernel = np.ones((1, 1), np.uint8)  # 用于腐蚀和膨胀操作的小核
        self.kernel2 = np.ones((2, 2), np.uint8)  # 用于膨胀操作的大核
        self.kernel3 = np.ones((1, 1), np.uint8)  # 用于腐蚀操作的小核
        # 定义白色在HSV色彩空间中的范围
        self.lower_white = np.array([0, 0, 200])
        self.upper_white = np.array([180, 25, 255])

    def create_white_mask(self, hsv):
        # 创建白色区域的掩码
        white_mask = cv2.inRange(hsv, self.lower_white, self.upper_white)
        return white_mask

    def process_frame(self, frame):
        try:
            # 将BGR图像转换为HSV色彩空间
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            white_mask = self.create_white_mask(hsv)
            # 使用掩码提取白色线条，并转换回RGB空间
            white_lines = cv2.bitwise_and(frame, frame, mask=white_mask)
            white_lines = cv2.cvtColor(white_lines, cv2.COLOR_HSV2BGR)
            # 对白色目标进行二值化处理
            gray = cv2.cvtColor(white_lines, cv2.COLOR_BGR2GRAY)
            _, binary_image = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY)
            
            # 对二值化后的图像进行形态学操作
            binary_image = cv2.morphologyEx(binary_image, cv2.MORPH_OPEN, self.kernel)
            binary_image = cv2.morphologyEx(binary_image, cv2.MORPH_CLOSE, self.kernel)
            binary_image = cv2.dilate(binary_image, self.kernel2, iterations=5)
            binary_image = cv2.erode(binary_image, self.kernel3, iterations=5)

            # 在新的窗口中显示处理后的二值图像
            cv2.imshow('Processed Binary Lines', binary_image)

            # 使用霍夫变换检测直线
            lines = cv2.HoughLinesP(binary_image, rho=1, theta=np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)

            # 创建一个新的全黑的二值图像，用于绘制检测到的线条
            # 这里我们使用与原始帧相同的大小，但是所有像素值都设置为0（黑色）
            line_image = np.zeros_like(binary_image)

            # 如果检测到直线，在line_image上绘制这些直线
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(line_image, (x1, y1), (x2, y2), 255, 2)  # 使用255表示白色线条

            # 在新的窗口中显示带有白色线条的二值化图像
            cv2.imshow('Binary Image with White Lines', line_image)


            # 等待按键，'q'键退出
            return cv2.waitKey(50) & 0xFF != ord('q')
        except Exception as e:
            print("处理帧时出现异常：", e)
            return False

    def run(self):
        if not self.video_capture.isOpened():
            print("错误：无法打开视频。")
            return
        try:
            while self.video_capture.isOpened():
                ret, frame = self.video_capture.read()
                if not ret:
                    print("无法接收帧（流结束？）。退出 ...")
                    break
                if not self.process_frame(frame):
                    break
        finally:
            self.video_capture.release()
            cv2.destroyAllWindows()

# 程序的入口点
if __name__ == '__main__':
    # 设置视频文件路径
    video_file = r'C:\Users\luojiatao\Desktop\ucar_ws\src\vision\001.mp4'  # 替换为你的视频文件路径
    # 创建LineFollower对象并运行主程序
    line_follower = LineFollower(video_file)
    line_follower.run()
