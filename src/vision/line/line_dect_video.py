import cv2
import numpy as np

# 定义一个跟踪线条的类
class LineFollower:
    def __init__(self, video_file):
        # 初始化视频捕获对象
        self.video_capture = cv2.VideoCapture(video_file)
        # 创建一个用于形态学操作的核
        self.kernel = np.ones((1, 1), np.uint8)
        # twist变量用于存储可能的控制指令，这里不使用

    # 处理每一帧的方法
    def process_frame(self, frame):
        try:
            # 将BGR图像转换为HSV色彩空间
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            # 定义白色在HSV色彩空间中的范围
            lower_white = np.array([0, 0, 200])
            upper_white = np.array([180, 25, 255])
            # 创建一个白色区域的掩码
            white_mask = cv2.inRange(hsv, lower_white, upper_white)
            # 应用开运算（先腐蚀后膨胀），去除噪声
            white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, self.kernel)
            # 应用闭运算（先膨胀后腐蚀），连接断裂的线条
            white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, self.kernel)
            # 膨胀操作，增强线条
            white_mask = cv2.dilate(white_mask, self.kernel, iterations=1)
            # 腐蚀操作，细化线条
            white_mask = cv2.erode(white_mask, self.kernel, iterations=1)
            # 使用掩码提取白色线条
            white_lines = cv2.bitwise_and(frame, frame, mask=white_mask)
            # 显示原始视频帧和处理后的帧
            cv2.imshow('Original Video', frame)
            cv2.imshow('White Lines', white_lines)
            # 等待按键，'q'键退出
            # 25ms的延迟确保大约40fps的帧率
            if cv2.waitKey(50) & 0xFF == ord('q'):
                return False  # 如果按下'q'，返回False以停止循环
        except Exception as e:
            print("处理帧时出现异常：", e)
            return False
        return True  # 继续循环

    # 主程序
    def main(self):
        # 如果视频无法打开，打印错误信息
        if not self.video_capture.isOpened():
            print("错误：无法打开视频。")
            return
        # 循环读取视频帧
        while self.video_capture.isOpened():
            ret, frame = self.video_capture.read()
            # 如果读取帧失败，退出循环
            if not ret:
                print("无法接收帧（流结束？）。退出 ...")
                break
            # 处理帧并检查是否应该继续
            if not self.process_frame(frame):
                break  # 如果process_frame返回False，则停止循环
        # 释放视频捕获对象
        self.video_capture.release()
        cv2.destroyAllWindows()

# 程序的入口点
if __name__ == '__main__':
    # 设置视频文件路径
    video_file = r'C:\Users\luojiatao\Desktop\ucar_ws\src\vision\001.mp4'  # 替换为你的视频文件路径
    # 创建LineFollower对象并运行主程序
    line_follower = LineFollower(video_file)
    line_follower.main()
