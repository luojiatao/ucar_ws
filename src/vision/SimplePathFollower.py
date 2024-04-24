import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import math

def path_callback(msg, local_path):
    local_path[:] = msg.poses  # 更新路径信息
    rospy.loginfo(f"Path updated with {len(local_path)} poses")

def compute_velocity_command(local_path, current_yaw):
    if not local_path:
        return Twist()

    target_pose = local_path[0].pose
    target_yaw = math.atan2(target_pose.position.y, target_pose.position.x)
    angle_error = normalize_angle(target_yaw - current_yaw)
    
    k = 1.0  # 位置控制增益
    k_theta = 0.5  # 旋转控制增益
    vx = k * target_pose.position.x
    vy = k * target_pose.position.y
    omega = k_theta * angle_error
    
    cmd = Twist()
    cmd.linear.x = vx
    cmd.linear.y = vy
    cmd.angular.z = omega
    return cmd

def normalize_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi

def follow_path(vel_pub, local_path, current_yaw):
    rate = rospy.Rate(10)  # 设置更新频率为10Hz
    while not rospy.is_shutdown():
        cmd_vel = compute_velocity_command(local_path, current_yaw)
        vel_pub.publish(cmd_vel)
        rospy.loginfo("Published velocity command")
        rate.sleep()

def main():
    rospy.init_node('simple_path_follower', anonymous=True)
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    local_path = []
    current_yaw = 0.0

    path_sub = rospy.Subscriber('/path', Path, lambda msg: path_callback(msg, local_path))

    try:
        follow_path(vel_pub, local_path, current_yaw)
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node terminated.")

if __name__ == '__main__':
    main()
