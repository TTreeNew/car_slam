import rclpy
from geometry_msgs.msg import PoseStamped, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from rclpy.duration import Duration
from autopatrol_interfaces.srv import SpeachText

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

class PatrolNode(BasicNavigator):
    def __init__(self,node):
        super().__init__(node)

        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        self.declare_parameter('init_point', [0.0, 0.0, 0.0])
        self.declare_parameter('target_points', [0.0, 0.0, 0.0])
        self.declare_parameter('image_save_path','')
        self.init_point_ = self.get_parameter('init_point').value
        self.target_points_ = self.get_parameter('target_points').value
        self.image_save_path_ = self.get_parameter('image_save_path').value

        self.speach_client_ = self.create_client(SpeachText, 'speach_text')

        self.image_subscriber_ = self.create_subscription(Image,'/camera_sensor/image_raw',self.image_callback,10)
        self.bridge = CvBridge()
        self.latest_image_ = None

        self.orb = cv2.ORB_create(nfeatures=500) 
            # 声明家具配置参数###
        self.declare_parameter('cylinder_name','')
        self.declare_parameter('cylinder_path','')
        self.declare_parameter('cabinet_name',"")
        self.declare_parameter('cabinet_path',"")
        self.declare_parameter('table_name',"")
        self.declare_parameter('table_path',"")
        self.declare_parameter('car_name',"")
        self.declare_parameter('car_path',"")

        # 构建家具配置字典###
        self.furniture_config = {
            'cylinder': {
                'template_path': self.get_parameter('cylinder_path').value,
                'chinese_name': self.get_parameter('cylinder_name').value
            },
            'cabinet': {
                'template_path': self.get_parameter('cabinet_path').value,
                'chinese_name': self.get_parameter('cabinet_name').value
            },
            'table': {
                'template_path': self.get_parameter('table_path').value,
                'chinese_name': self.get_parameter('table_name').value
            },
            'car': {
                'template_path': self.get_parameter('car_path').value,
                'chinese_name': self.get_parameter('car_name').value
            }
        }
        # 加载模板特征 ###
        self.templates = {}
        for eng_name, config in self.furniture_config.items():
            # 1. 读取图片
            template_img = cv2.imread(config['template_path'], cv2.IMREAD_GRAYSCALE)  # 统一用灰度图
            if template_img is None:
                self.get_logger().error(f"模板 {eng_name} 加载失败，路径：{config['template_path']}")
                continue  # 跳过当前模板

            # 2. 使用成员变量self.orb提取特征（确保与识别时一致）
            kp, des = self.orb.detectAndCompute(template_img, None)
            if des is None:  # 新增检查
                self.get_logger().error(f"模板 {eng_name} 未提取到特征")
                continue

            # 3. 存储特征
            self.templates[eng_name] = {
                'des': des,
                'ch_name': config['chinese_name']
            }
            self.get_logger().info(f"模板 {eng_name} 加载成功，特征数：{len(kp)}")


        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.detection_threshold = 50  # 最低匹配特征点数


    def recognize_furniture(self, cv_image):     ###
        """执行识别并返回中文名称"""
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        kp, des = self.orb.detectAndCompute(gray, None)
        
        if des is None:  # 无特征可提取
            return None
            
        for eng_name, template in self.templates.items():
            matches = self.matcher.match(template['des'], des)
            if len(matches) >= self.detection_threshold:
                return template['ch_name']
        return None

    def image_callback(self, msg):
        self.latest_image_ = msg

    def save_image(self):
        """保存+识别图像"""
        if self.latest_image_ is not None:
            pose = self.get_current_pose()
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image_)
            cv2.imwrite(f'{self.image_save_path_}image_{pose.translation.x:3.2f},{pose.translation.y:3.2f}.jpg', cv_image)

        try:
            # 识别功能
            chinese_name = self.recognize_furniture(cv_image)
            if chinese_name:
                self.speach_text(f"识别到{chinese_name}")
            else:
                self.speach_text("无法识别")
                
        except Exception as e:
            self.get_logger().error(f"记录识别失败: {str(e)}")
            self.speach_text("识别异常")


    def speach_text(self, text):
        while not self.speach_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待语音服务可用...')
        request = SpeachText.Request()
        request.text = text
        future = self.speach_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            result = SpeachText.Response()
            if result:
                self.get_logger().info(f'语音合成成功: {text}')
            else:
                self.get_logger().error('语音合成失败')
        else:
            self.get_logger().error('语音服务调用失败')


    def get_pose_by_xyyaw(self, x, y, yaw):
        pose=PoseStamped()
        pose.header.frame_id = 'map'
       ## pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        rotation_quat = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = rotation_quat[0]
        pose.pose.orientation.y = rotation_quat[1]
        pose.pose.orientation.z = rotation_quat[2]
        pose.pose.orientation.w = rotation_quat[3]
        return pose
        

    def init_robot_pose(self):
        self.init_point_ = self.get_parameter('init_point').value
        self.setInitialPose(self.get_pose_by_xyyaw(self.init_point_[0], self.init_point_[1], self.init_point_[2]))
        self.waitUntilNav2Active()

    def get_target_points(self):
        points = []
        self.target_points_ = self.get_parameter('target_points').value
        for index in range(int(len(self.target_points_)/3)):
            x = self.target_points_[index*3]
            y = self.target_points_[index*3+1]
            yaw = self.target_points_[index*3+2]
            points.append([x, y, yaw])
            self.get_logger().info(f'获取目标点：{index+1}:{x}, {y}, {yaw}')
        return points
##points需要转化成target_pose
    def nav_to_pose(self, target_pose):
        self.waitUntilNav2Active()
        self.goToPose(target_pose)
        while not self.isTaskComplete():
            feedback = self.getFeedback()
            if feedback:
            ###    self.get_logger().info(f'导航反馈：{feedback}')   #feedback内容太多了吧
                self.get_current_pose()
                self.get_logger().info(f'剩余距离：{feedback.distance_remaining}')

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('导航成功')
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('导航取消')
        elif result == TaskResult.FAILED:
            self.get_logger().error('导航失败')
        else:
            self.get_logger().error('未知错误')

    def get_current_pose(self):
        try:
            tf = self.tf_buffer_.lookup_transform('map', 'base_footprint', rclpy.time.Time(seconds=0), rclpy.time.Duration(seconds=1))
            transform = tf.transform
            rotation_euler = euler_from_quaternion([transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w])
            self.get_logger().info(f'位移：{transform.translation}, 旋转欧拉角：{rotation_euler},旋转四元数:{transform.rotation}')
            return transform
        except Exception as e:
            self.get_logger().warn(f'获取当前位姿失败: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode("patrol_node")
    node.init_robot_pose()
    points = node.get_target_points()
    while rclpy.ok():
        for point in points:
            x,y,yaw = point[0], point[1], point[2]
            node.speach_text(f'正在前往目标点{x},{y},{yaw}')
            node.get_logger().info(f'正在前往目标点{x},{y},{yaw}')
            target_pose = node.get_pose_by_xyyaw(x,y,yaw)
            node.nav_to_pose(target_pose)
            node.save_image()            
            node.speach_text(f'已到达目标点{x},{y},{yaw}, 图像已保存')
    rclpy.shutdown()

