import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import yaml
import os
from geometry_msgs.msg import PoseStamped

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')

        # Path 퍼블리셔 설정
        self.path_publisher = self.create_publisher(Path, '/path', 10)

        # 타이머 설정: 5초마다 Path 퍼블리시
        self.publish_path()
        self.timer = self.create_timer(5.0, self.publish_path)

    def load_path_from_yaml(self, file_path):
        # YAML 파일에서 경로 데이터를 불러오기
        with open(file_path, 'r') as file:
            loaded_path_data = yaml.safe_load(file)
        return loaded_path_data

    def publish_path(self):
        # YAML 파일에서 경로 불러오기
        file_path = '/home/kdw/dataset/initial_pose_data.yaml'
        if not os.path.exists(file_path):
            self.get_logger().warn('YAML file not found. Make sure the file path_data.yaml exists.')
            return

        loaded_path_data = self.load_path_from_yaml(file_path)

        # Path 메시지 생성
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # 불러온 경로 데이터를 Path 메시지로 변환
        for pose_data in loaded_path_data:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = pose_data['position']['x']
            pose.pose.position.y = pose_data['position']['y']
            pose.pose.position.z = pose_data['position']['z']
            pose.pose.orientation.x = pose_data['orientation']['x']
            pose.pose.orientation.y = pose_data['orientation']['y']
            pose.pose.orientation.z = pose_data['orientation']['z']
            pose.pose.orientation.w = pose_data['orientation']['w']
            path_msg.poses.append(pose)

        # Path 메시지 퍼블리시
        self.path_publisher.publish(path_msg)
        self.get_logger().info('Published path from YAML file.')

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
