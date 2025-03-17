import math
import rclpy
from rclpy.node import Node
import tf2_ros
from std_msgs.msg import Float32,String
from geometry_msgs.msg import Point  # 3D 좌표를 위한 메시지 타입
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion
import numpy as np
class DistanceWorldChildPublisher(Node):
    def __init__(self):
        super().__init__('distance_world_child_publisher')
        # TF를 조회하기 위한 Buffer와 Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # 거리 정보를 발행할 Publisher
        #self.dist_pub = self.create_publisher(Float32, 'distance_world_child', 10)
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        # 다른 노드에서 좌표를 받아올 Subscriber
        self.coord_sub = self.create_subscription(
            Point,  # 3D 좌표를 위한 메시지 타입
            '/person_detection',  # 다른 노드가 발행하는 토픽 이름
            self.coord_callback,
            10
        )
        # 외부 좌표 저장 변수 초기화
        self.external_coords = None
        # 주기적으로 TF를 조회 (0.1초마다)
        self.timer = self.create_timer(0.1, self.timer_callback)
    def coord_callback(self, msg):
        # 다른 노드에서 좌표를 받아왔을 때 실행되는 콜백
        self.external_coords = msg
        self.get_logger().info(f"Received external coordinates: ({msg.x}, {msg.y}, {msg.z})")
        # 좌표를 받았으니 TF 변환 및 거리 계산 실행
        self.calculate_and_publish_distance()
    def timer_callback(self):
        # 외부 좌표가 있는지 확인만 하고, 실제 계산은 하지 않음
        if self.external_coords is None:
            self.get_logger().info("Waiting for external coordinates...")
    def calculate_and_publish_distance(self):
        # 외부 좌표가 없으면 계산하지 않음
        if self.external_coords is None:
            return
        try:
            # 'map'에서 'camera_link'으로의 변환 조회
            x,y,z,qx,qy,qz,qw=0,0,0,0,0,0,0
            transform = self.tf_buffer.lookup_transform(
                'map',
                'camera_link',
                rclpy.time.Time()
            )
            # 변환으로부터 x, y, z 추출
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            qx= transform.transform.rotation.x
            qy= transform.transform.rotation.y
            qz= transform.transform.rotation.z
            qw= transform.transform.rotation.w
            _,_,yaw = euler_from_quaternion([qx,qy,qz,qw])
            origin=np.array([x,y])
            
            fx=227.34
            fy=227.34
            cx=163.6
            cy=123.12
            # 외부 좌표를 더함
            x_local= self.external_coords.z-0.8
            y_local= -(self.external_coords.x-cx)/fx
            # x_local= self.external_coords.x-0.8
            # print("xxxxxxxxxxxxxxx", x,  yaw)
            # y_local= self.external_coords.y
            x1=x_local*math.cos(yaw)-y_local*math.sin(yaw)
            print("xxxxxxxxxxxxxxx", x,  yaw)
            y1=x_local*math.sin(yaw)+y_local*math.cos(yaw)
            x+=x1
            print("yyyyyyyyyyyyyy", x,  yaw)
            y+=y1
            # 3차원 유클리드 거리 계산
            distance = math.sqrt(x**2 + y**2 + z**2)
            # 메시지 생성 및 발행
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = 0.0
        # yaw 값을 이용해 회전(쿼터니언)을 계산
        # 회전이 없는 상태: yaw=0 -> quaternion: (x=0, y=0, z=0, w=1)
            
            msg.pose.orientation.x = qx
            msg.pose.orientation.y = qy
            msg.pose.orientation.z = qz
            msg.pose.orientation.w = qw
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published distance: {distance} (combined coordinates: {x}, {x_local}, {z})")
        except Exception as e:
            # 아직 TF가 준비되지 않았거나, lookup 실패할 경우 예외 발생
            self.get_logger().warn(f"Could not get transform: {e}")
def main(args=None):
    rclpy.init(args=args)
    node = DistanceWorldChildPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()