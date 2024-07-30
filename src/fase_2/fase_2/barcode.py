# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from std_msgs.msg import String # String message type for publishing barcode data
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from pyzbar import pyzbar

# Function to decode barcodes in a frame
def decode_barcode(frame):
    barcodes = pyzbar.decode(frame)
    for barcode in barcodes:
        (x, y, w, h) = barcode.rect
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        barcode_data = barcode.data.decode('utf-8')
        barcode_type = barcode.type
        text = f'{barcode_data} ({barcode_type})'
        cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    return frame, barcodes

class ImageSubscriber(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('image_subscriber')
        
        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            Image, 
            'camera', 
            self.listener_callback, 
            10)
        self.subscription # prevent unused variable warning
        
        # Create a publisher for barcode data
        self.publisher_ = self.create_publisher(String, 'barcode_data', 10)
        
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()
    
    def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        self.get_logger().info('Receiving video frame')
    
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding="bgr8")
        
        # Decode barcodes
        frame, barcodes = decode_barcode(current_frame)
        
        # Publish barcode data
        for barcode in barcodes:
            barcode_data = barcode.data.decode('utf-8')
            barcode_type = barcode.type
            barcode_msg = String()
            barcode_msg.data = f'{barcode_data} ({barcode_type})'
            self.publisher_.publish(barcode_msg)
            self.get_logger().info(f'Detected {barcode_type} barcode: {barcode_data}')
        
         # Resize the frame
        resized_frame = cv2.resize(frame, (880, 680))  # Adjust the size as needed
        
        # Show Results
        cv2.imshow('Detected Frame',  resized_frame)    
        cv2.waitKey(1)

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    
    # Create the node
    image_subscriber = ImageSubscriber()
    
    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()
    
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()