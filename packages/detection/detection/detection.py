import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO
import cv2
import numpy as np
import threading
from threading import Lock

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Parameters
        self.declare_parameter('model', 'yolo11n.pt')
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('rtsp_url', 'rtsp://localhost:8554/cam')
        self.declare_parameter('process_fps', 1.0)
        self.declare_parameter('jpeg_quality', 90)
        
        model_name = self.get_parameter('model').value
        self.confidence = self.get_parameter('confidence').value
        device = self.get_parameter('device').value
        self.rtsp_url = self.get_parameter('rtsp_url').value
        process_fps = self.get_parameter('process_fps').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        
        # Load YOLO model
        self.get_logger().info(f'Loading model {model_name} ...')
        self.model = YOLO(model_name, task='detect')
        self.model.to(device)
        
        # Thread-safe storage for the latest frame
        self.latest_frame = None
        self.frame_lock = Lock()
        self.running = True
        
        # Start background thread to read RTSP frames
        self.cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open RTSP stream: {self.rtsp_url}')
            raise RuntimeError('Cannot connect to RTSP stream')
        self.get_logger().info(f'Connected to RTSP stream: {self.rtsp_url}')
        
        self.read_thread = threading.Thread(target=self._frame_reader, daemon=True)
        self.read_thread.start()
        
        # Publisher for compressed annotated image (no /detections topic)
        self.annotated_pub = self.create_publisher(
            CompressedImage,
            '/camera/image_annotated_compressed',
            10
        )
        
        # Timer for frame processing
        timer_period = 1.0 / process_fps
        self.processing_timer = self.create_timer(timer_period, self.process_frame)
        
        self.get_logger().info(f'YOLO detector ready, processing at {process_fps} Hz')

    def _frame_reader(self):
        """Background thread: continuously reads frames from RTSP and stores the latest."""
        frame_read_errors = 0
        max_errors = 5
        
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                frame_read_errors += 1
                self.get_logger().warn(f'Frame read error #{frame_read_errors}')
                if frame_read_errors >= max_errors:
                    self.get_logger().warn('Reconnecting to RTSP stream...')
                    self.cap.release()
                    self.cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
                    frame_read_errors = 0
                    if not self.cap.isOpened():
                        self.get_logger().error('Reconnection failed')
                continue
            
            frame_read_errors = 0
            
            # Store the latest frame in a thread-safe way
            with self.frame_lock:
                self.latest_frame = frame

        self.cap.release()

    def process_frame(self):
        """Timer callback: runs detection on the latest frame and publishes compressed annotated image."""
        # Get the latest frame
        with self.frame_lock:
            frame = self.latest_frame
            if frame is None:
                return
        
        # Run inference
        results = self.model(frame, conf=self.confidence, verbose=False)
        
        # Create annotated image (BGR numpy array)
        annotated = results[0].plot() if len(results) > 0 else frame
        
        # Build CompressedImage message
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        msg.format = 'jpeg'
        
        # Encode annotated image as JPEG
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        success, buffer = cv2.imencode('.jpg', annotated, encode_param)
        if success:
            msg.data = np.array(buffer).tobytes()
            self.annotated_pub.publish(msg)
        else:
            self.get_logger().warn('Failed to encode image to JPEG')
            return
        
        # Optional logging
        num_detections = 0
        if len(results) > 0:
            for result in results:
                num_detections += len(result.boxes)
            if num_detections > 0:
                class_names = []
                for result in results:
                    for box in result.boxes:
                        class_names.append(self.model.names[int(box.cls[0])])
                self.get_logger().debug(f'Detected {num_detections}: {class_names}')
            else:
                self.get_logger().debug('No detections')
        else:
            self.get_logger().debug('No detection results')

    def destroy_node(self):
        """Override destroy_node to cleanly stop the background thread."""
        self.running = False
        if self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
