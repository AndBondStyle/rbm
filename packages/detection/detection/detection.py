import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import Header
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Параметры
        self.declare_parameter('model', 'yolo11n.pt')
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('device', 'cpu')  # или 'cuda:0' для GPU
        
        model_name = self.get_parameter('model').value
        self.confidence = self.get_parameter('confidence').value
        device = self.get_parameter('device').value
        
        # Загружаем модель
        self.get_logger().info(f'Загрузка модели {model_name}...')
        self.model = YOLO(model_name, task='detect')
        self.model.to(device)
        
        # cv_bridge
        self.bridge = CvBridge()
        
        # Подписка на камеру
        self.subscription = self.create_subscription(
            Image,
            '/camera_node/image_raw',
            self.image_callback,
            10
        )
        
        # Публикация детекций
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/detections',
            10
        )
        
        # Публикация изображения с разметкой
        self.annotated_pub = self.create_publisher(
            Image,
            '/camera/image_annotated',
            10
        )

        self.process_timer = self.create_timer(0.5, self.process)
        self.last_msg = None
        
        self.get_logger().info('YOLO detector готов!')

    def image_callback(self, msg):
        self.last_msg = msg
    
    def process(self):
        msg = self.last_msg
        if msg is None: return

        # Конвертируем ROS Image → OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Детекция
        results = self.model(cv_image, conf=self.confidence, verbose=False)
        
        # Создаём сообщение с детекциями
        detections_msg = Detection2DArray()
        detections_msg.header = msg.header
        
        for result in results:
            for box in result.boxes:
                detection = Detection2D()
                
                # Bounding box
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                detection.bbox.center.position.x = (x1 + x2) / 2
                detection.bbox.center.position.y = (y1 + y2) / 2
                detection.bbox.size_x = x2 - x1
                detection.bbox.size_y = y2 - y1
                
                # Класс и уверенность
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = self.model.names[int(box.cls[0])]
                hypothesis.hypothesis.score = float(box.conf[0])
                detection.results.append(hypothesis)
                
                detections_msg.detections.append(detection)
        
        # Публикуем детекции
        self.detection_pub.publish(detections_msg)
        
        # Публикуем аннотированное изображение
        annotated = results[0].plot()
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)
        
        # Логируем
        num_detections = len(detections_msg.detections)
        if num_detections > 0:
            classes = [d.results[0].hypothesis.class_id for d in detections_msg.detections]
            self.get_logger().info(f'Обнаружено {num_detections}: {classes}')


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
