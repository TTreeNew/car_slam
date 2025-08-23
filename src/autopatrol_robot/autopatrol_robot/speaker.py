import rclpy
from rclpy.node import Node
from autopatrol_interfaces.srv import SpeachText
import espeakng

class Speaker(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.speach_service = self.create_service(SpeachText, 'speach_text', self.speach_callback)
        self.speaker = espeakng.Speaker()
        self.speaker.voice = 'zh'

    def speach_callback(self, request, response):
        self.get_logger().info(f'接收到语音请求: {request.text}')
        self.speaker.say(request.text)
        self.speaker.wait()
        response.result = True
        return response
    
def main(args=None):
    rclpy.init(args=args)
    node = Speaker("speaker_node")
    rclpy.spin(node)
    rclpy.shutdown()