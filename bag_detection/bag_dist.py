#!/usr/bin/env python3

import rospy
import cv2
import pyrealsense2 as rs
import numpy as np
from ultralytics import YOLO
from std_msgs.msg import Float32

def main():
    rospy.init_node('bag_detection_node')
    distance_pub = rospy.Publisher('/bag_distance', Float32, queue_size=10)
    angle_pub = rospy.Publisher('/bag_angle', Float32, queue_size=10)

    # Configure RealSense camera
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    # Load the trained model
    model = YOLO("/home/leongonn/r1_wiki_ws/src/skeletal_cam/src/bag_detection/best2.pt")

    # Horizontal FOV of the RealSense D435 camera (degrees)
    HORIZONTAL_FOV = 87.2
    IMAGE_WIDTH = 640  # Image width of the color stream

    try:
        while not rospy.is_shutdown():
            # Get frames from the camera
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                rospy.logwarn("Failed to retrieve frames from the camera")
                continue
            
            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())

            # Use the model to detect objects in the frame
            results = model(color_image)
            detections = results[0]

            highest_confidence = 0.0
            best_box = None
            best_score = None
            best_center_x = None
            best_center_y = None

            for det in detections.boxes:
                if det.conf > 0.7:
                    confidence = det.conf[0]
                    if confidence > highest_confidence:
                        highest_confidence = confidence
                        best_box = det.xyxy[0].tolist()
                        best_score = confidence
                        best_center_x = int((best_box[0] + best_box[2]) // 2)
                        best_center_y = int((best_box[1] + best_box[3]) // 2)

            if best_box:
                # Get the center point of the bounding box with the highest confidence
                x1, y1, x2, y2 = map(int, best_box)
                
                # Get the depth value at the center of the bounding box
                depth_value = depth_frame.get_distance(best_center_x, best_center_y)

                # Calculate the angle of the bag relative to the camera's center
                image_center_x = IMAGE_WIDTH // 2
                pixel_offset = best_center_x - image_center_x
                angle = (pixel_offset / IMAGE_WIDTH) * HORIZONTAL_FOV
                angle_deg = angle

                # Publish the distance and angle
                distance_pub.publish(depth_value)
                angle_pub.publish(angle_deg)
                rospy.loginfo(f'Highest Confidence Bag - Distance: {depth_value:.2f} meters')
                rospy.loginfo(f'Highest Confidence Bag - Angle: {angle_deg:.2f} degrees')

                # Annotate the frame
                label = "Bag"
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(color_image, f'{label} Conf: {best_score:.2f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(color_image, f'Distance: {depth_value:.2f}m', (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(color_image, f'Angle: {angle_deg:.2f} deg', (x1, y2 + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Display the annotated frame
            cv2.imshow('Bag Detection', color_image)

            # Exit on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

