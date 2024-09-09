import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO  
from std_msgs.msg import String


class MultiImageSubscriber(Node):

    def __init__(self):
        super().__init__('multi_image_subscriber')
        
        self.bridge = CvBridge()
        
        self.right_cam_subscriber = self.create_subscription(
            CompressedImage,
            '/right_camera/image_raw/compressed',
            self.right_cam_callback,
            10)
        self.left_cam_subscriber = self.create_subscription(
            CompressedImage,
            '/left_camera/image_raw/compressed',
            self.left_cam_callback,
            10)
        self.front_cam_subscriber = self.create_subscription(
            CompressedImage,
            '/front_camera/image_raw/compressed',
            self.front_cam_callback,
            10)
        self.rear_cam_subscriber = self.create_subscription(
            CompressedImage,
            '/rear_camera/image_raw/compressed',
            self.rear_cam_callback,
            10)
        
        self.right_yolo_publisher = self.create_publisher(CompressedImage, "/right_camera/yolo", 10)
        self.left_yolo_publisher = self.create_publisher(CompressedImage, "/left_camera/yolo", 10)
        self.front_yolo_publisher = self.create_publisher(CompressedImage, "/front_camera/yolo", 10)
        self.rear_yolo_publisher = self.create_publisher(CompressedImage, "/rear_camera/yolo", 10)

        self.target_coords_publisher = self.create_publisher(String, "/target_coordinates", 10)

        self.front_yolo_subscriber = self.create_subscription(
            CompressedImage,
            '/front_camera/yolo',
            self.front_yolo_callback,
            10
        )

        self.model = YOLO('best.pt')

        self.get_logger().info('Multi Image Subscriber Node has been started.')

    def right_cam_callback(self, msg):
        #self.detect_target(msg, "Right Camera")
        pass

    def left_cam_callback(self, msg):
        #self.detect_target(msg, "Left Camera")
        pass

    def front_cam_callback(self, msg):
        self.detect_target(msg, self.front_yolo_publisher)

    def rear_cam_callback(self, msg):
        #self.detect_target(msg, "Rear Camera")
        pass

    def detect_target(self, msg, img_publisher):

        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        #cv2.imshow("Front Image YOLO", cv_image)
        #cv2.waitKey(1)

        results = self.model(cv_image)
        cars = []
        coords = String()

        for result in results:
            boxes = result.boxes.cpu().numpy()
            for box in boxes:
                if box.cls[0] == 1 and box.conf[0] > 0.6:
                    r = box.xyxy[0].astype(int) 
                    coords.data = f"{str(r[0])},{str(r[2])}"
                    cars.append(r)
                    class_id = int(box.cls[0]) 
                    #class_name = self.model.names[class_id]  
                    cv2.rectangle(cv_image, r[:2], r[2:], (0, 0, 255), 3)  
                    cv2.putText(cv_image, "Target", ((r[0] + r[2]) // 3, r[1]-10), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,255), 2)

        _, encoded_image = cv2.imencode('.jpg', cv_image)

        compressed_msg = CompressedImage()
        compressed_msg.header.stamp = self.get_clock().now().to_msg()
        compressed_msg.format = "jpeg"
        compressed_msg.data = np.array(encoded_image).tobytes()

        img_publisher.publish(compressed_msg)
        self.target_coords_publisher.publish(coords)


    def front_yolo_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        cv2.imshow("Front Image YOLO", cv_image)
        cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    node = MultiImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
