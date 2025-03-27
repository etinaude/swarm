import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

# openCV
import cv2
from inference import InferencePipeline
import numpy as np
import cv_bridge


class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        # self.subscription = self.create_subscription(
        #     CompressedImage,
        #     '/image_raw/compressed',
        #     self.listener_callback,
        #     10)
        # self.subscription  # prevent unused variable warning


        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.listener_callback,
            10)

        self.subscription  # prevent unused variable warning

        # OpenCV's HOG-based people detector
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        # CV bridge to convert ROS 2 images to OpenCV format
        self.bridge = cv_bridge.CvBridge()

    def listener_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Detect people in the frame
        boxes, _ = self.hog.detectMultiScale(frame, winStride=(8, 8), padding=(4, 4), scale=1.05)

        # Process detected people
        for (x, y, w, h) in boxes:
            center_x, center_y = x + w // 2, y + h // 2
            self.get_logger().info(f"Person detected at: ({center_x}, {center_y})")

            # Draw bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

        # Show the frame (for debugging)
        cv2.imshow("Person Detection", frame)
        cv2.waitKey(1)


        # # if a person is in the image print "Person detected"
        # # if not print "No person detected"
        # np_arr = np.fromstring(msg.data, np.uint8)
        # image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # cv2.imshow('image', image_np)
        # cv2.waitKey(1)


        # self.get_logger().info('data')
        # # self.get_logger().info('I heard: "%s"' % msg.data)



# def my_sink(result, video_frame):
#     if result.get("output_image"): # Display an image from the workflow response
#         cv2.imshow("Workflow Image", result["output_image"].numpy_image)
#         cv2.waitKey(1)
#     print(result) # do something with the predictions of each frame


# # initialize a pipeline object
# pipeline = InferencePipeline.init_with_workflow(
#     api_key="t1yLFChZcU6Hy55rgvjz",
#     workspace_name="bricks-gakvj",
#     workflow_id="detect-count-and-visualize",
#     video_reference=0,
#     max_fps=30,
#     on_prediction=my_sink
# )
# pipeline.start()
# pipeline.join()


def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()