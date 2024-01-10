import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
from .params_parser import ParamParser
import cv2
import numpy as np

class PerspectiveTransformNode(Node):
    def __init__(self):
        super().__init__('node')
        self.bridge = CvBridge()
        #Subscribe to topic
        self.subscriptionImageCompressed = self.create_subscription(
          CompressedImage, '/image/compressed', self.compressed_callback, 10) 
        self.subscriptionImageRaw = self.create_subscription(
            Image,'/image', self.raw_callback, 10)
        self.occupancy_grid_publisher = self.create_publisher(
            OccupancyGrid, '/occupancy_grid', 10)
        self.config = self.load_config()
        print(self.config)
            
    def load_config(self):
        param_parser = ParamParser("/home/randytjoa/autopilot/vision-service/src/vision_mapping/config/config.yaml")
        return param_parser.parse()
        
    def compressed_callback(self,msg):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.image_callback(frame)

    def raw_callback(self,msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.image_callback(frame)


    def image_callback(self, frame):
        # Grid width and height determined
        grid_width, grid_height, camera_view, birdseye_view, mask_view, grid_view  = self.config.grid_width, self.config.grid_height, self.config.camera_view, self.config.birdseye_view, self.config.mask_view, self.config.grid_view
        resolution_x = 1280 // grid_width
        resolution_y = 720 // grid_height
        
        # Initialisation of empty grid
        occupancy_grid = np.zeros((grid_height, grid_width), dtype=np.uint8)

        # Selecting Coordinates
        tl = (318, 30)  # red
        bl = (152, 675)  # yellow
        tr = (962, 30)  # purple
        br = (1128, 675)  # blue

        cv2.circle(frame, tl, 5, (0, 0, 255), -1)
        cv2.circle(frame, bl, 5, (0, 255, 255), -1)
        cv2.circle(frame, tr, 5, (255, 0, 255), -1)
        cv2.circle(frame, br, 5, (255, 255, 0), -1)

        # Apply Geometrical Transformation
        pts1 = np.float32([tl, tr, bl, br])
        pts2 = np.float32([(0, 0), (1280, 0), (0, 720), (1280, 720)])
        
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        transformed_frame = cv2.warpPerspective(frame, matrix, (1280, 720))

        # Set threshold for green color
        lower_green = np.array([self.config.profiles[0].hue_low, self.config.profiles[0].sat_low, self.config.profiles[0].val_low], dtype=np.uint8)
        upper_green = np.array([self.config.profiles[0].hue_up, self.config.profiles[0].sat_up, self.config.profiles[0].val_up], dtype=np.uint8)

        # Convert to HSV format
        hsv_transformed_frame = cv2.cvtColor(transformed_frame, cv2.COLOR_BGR2HSV)

        # Create a mask based on the threshold
        green_mask_transformed = cv2.inRange(hsv_transformed_frame, lower_green, upper_green)

        # Apply the mask to the transformed frame
        result_transformed_frame = cv2.bitwise_and(transformed_frame, transformed_frame, mask=green_mask_transformed)

       # Convert the result to grayscale
        result_gray = cv2.cvtColor(result_transformed_frame, cv2.COLOR_BGR2GRAY)

        # Threshold the grayscale image
        _, thresholded = cv2.threshold(result_gray, 1, 255, cv2.THRESH_BINARY)

        # Resize thresholded image to match the grid size
        thresholded_resized = cv2.resize(thresholded, (grid_width, grid_height), interpolation=cv2.INTER_NEAREST)

        # Update the occupancy grid
        occupancy_grid = thresholded_resized

        
        # Update de occupancy grid met de geschatte positie en oriëntatie
        occupancy_grid_msg = self.create_occupancy_grid_message(thresholded_resized)
        self.occupancy_grid_publisher.publish(occupancy_grid_msg)
    
        # Display the results
        if camera_view == True:
            cv2.imshow("Frame", frame)
        if birdseye_view == True:
            cv2.imshow("transformed_frame Bird's Eye View", transformed_frame)
        if mask_view == True:
            cv2.imshow("Green Mask Transformed", green_mask_transformed)
        
        #Resize and show occupancy grid
        if grid_view == True:
            cv2.namedWindow("Occupancy grid", cv2.WINDOW_NORMAL)
            cv2.imshow("Occupancy grid", thresholded_resized)
            cv2.resizeWindow("Occupancy grid", 800,800)
        
        if cv2.waitKey(1) == 27:
            self.get_logger().info('Close...')
            cv2.destroyAllWindows()

    def create_occupancy_grid_message(self, thresholded_resized):
        occupancy_grid_msg = OccupancyGrid()
        occupancy_grid_msg.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid_msg.header.frame_id = 'map'  # Frame ID
        occupancy_grid_msg.info.resolution = 1.0  # Resolution of each grid cel
        occupancy_grid_msg.info.width = thresholded_resized.shape[1]  # Width of grid
        occupancy_grid_msg.info.height = thresholded_resized.shape[0]  # Height of grid
        
        #Map values to the range [-1,100]
        occupancy_grid_data = np.zeros_like(thresholded_resized, dtype=np.int8)
        occupancy_grid_data[thresholded_resized == 0] = 0 #Free space
        occupancy_grid_data[thresholded_resized == 255] = 100 #Occupied space
        occupancy_grid_data[thresholded_resized == 127] = -1 #Unkown space

        # Convert binarized(thresholded) image to 1D list
        occupancy_grid_data_flattened = occupancy_grid_data.flatten().astype(np.int8).tolist()
        occupancy_grid_msg.data = occupancy_grid_data_flattened

        return occupancy_grid_msg

def main(args=None):
    rclpy.init(args=args)
    node = PerspectiveTransformNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
