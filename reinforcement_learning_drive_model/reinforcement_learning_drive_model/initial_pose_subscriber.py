import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
import yaml
import os

class InitialPoseSubscriber(Node):
    def __init__(self):
        super().__init__('initial_pose_subscriber')
        self.initial_pose_data = []  # 데이터를 저장할 배열
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'initialpose',
            self.pose_callback,
            10
        )
        
        # 마커 퍼블리셔 생성
        self.marker_pub = self.create_publisher(Marker, 'initial_pose_marker', 10)
        self.marker_id = 0  # 각 마커에 고유 ID를 부여하기 위한 ID
        
        self.get_logger().info("Subscribed to initial_pose topic")

    def pose_callback(self, msg):
        # position과 orientation 데이터를 배열로 저장
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # 배열 형태로 변환
        pose_data = {
            'position': {
                'x': position.x,
                'y': position.y,
                'z': position.z
            },
            'orientation': {
                'x': orientation.x,
                'y': orientation.y,
                'z': orientation.z,
                'w': orientation.w
            }
        }

        # 배열에 추가
        self.initial_pose_data.append(pose_data)
        self.get_logger().info(f"Received initial pose: {pose_data}")

        # 마커 생성 및 퍼블리시
        self.publish_marker(position)

    def publish_marker(self, position):
        # 새로운 마커 설정
        marker = Marker()
        marker.header.frame_id = "map"  # 마커의 기준 프레임 설정
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "initial_pose"
        marker.id = self.marker_id  # 고유 ID 설정
        self.marker_id += 1  # 다음 마커를 위한 ID 증가

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = position.x
        marker.pose.position.y = position.y
        marker.pose.position.z = position.z
        marker.pose.orientation.w = 1.0  # 기본 orientation

        # 마커 크기 및 색상 설정
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # 퍼블리시
        self.marker_pub.publish(marker)
        self.get_logger().info(f"Published marker at ({position.x}, {position.y}, {position.z})")

    def save_to_yaml(self, file_path):
        # YAML 파일로 저장
        with open(file_path, 'w') as file:
            yaml.dump(self.initial_pose_data, file)
        self.get_logger().info(f"Data saved to {file_path}")

def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 종료 시 YAML 파일로 저장
        yaml_file_path = os.path.join(os.path.expanduser('~'), 'initial_pose_data.yaml')
        node.save_to_yaml(yaml_file_path)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

