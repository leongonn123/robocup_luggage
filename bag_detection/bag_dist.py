#!/usr/bin/env python3

import rospy
import cv2
import pyrealsense2 as rs
import numpy as np
from ultralytics import YOLO
from std_msgs.msg import Float32
import threading
import queue
from std_msgs.msg import Bool

# Function to rotate the image by the given angle (degrees)
def rotate_image(image, angle):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    return cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)

# Thread function to capture frames from the camera
def capture_frames(pipeline, frame_queue):
    while not rospy.is_shutdown():
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            rospy.logwarn("Failed to retrieve frames from the camera")
            continue

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Rotate images to correct for upside-down camera installation
        depth_image = rotate_image(depth_image, 180)
        color_image = rotate_image(color_image, 180)

        if not frame_queue.full():
            frame_queue.put((depth_image, color_image))
            
# Corrected callback function without 'self'
def reach_distance_callback(msg):
    if msg.data:  # If the message is True
        rospy.loginfo("Target distance reached. Shutting down node.")
        rospy.signal_shutdown("Shutting down due to reaching the target distance.")


# Thread function to process frames
def process_frames(frame_queue, distance_pub, angle_pub, model, HORIZONTAL_FOV, IMAGE_WIDTH):
    while not rospy.is_shutdown():
        if not frame_queue.empty():
            depth_image, color_image = frame_queue.get()

            # Consider processing every nth frame to reduce load
            results = model(color_image, stream=False)  # Disable streaming to reduce load

            highest_confidence = 0.0
            best_box = None
            best_score = None

            for det in results[0].boxes:
                if det.conf > 0.8:
                    confidence = det.conf[0]
                    if confidence > highest_confidence:
                        highest_confidence = confidence
                        best_box = det.xyxy[0].tolist()
                        best_score = confidence

            if best_box:
                x1, y1, x2, y2 = map(int, best_box)
                best_center_x = (x1 + x2) // 2
                best_center_y = (y1 + y2) // 2
                depth_value = depth_image[best_center_y, best_center_x]
                
                # Calculate the angle
                image_center_x = IMAGE_WIDTH // 2
                pixel_offset = best_center_x - image_center_x
                angle = (pixel_offset / IMAGE_WIDTH) * HORIZONTAL_FOV
                angle_deg = angle

                # Publish the distance and angle
                distance_pub.publish(depth_value)
                angle_pub.publish(angle_deg)

                # Annotate the frame (consider skipping this for every frame)
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(color_image, f'Bag Conf: {best_score:.2f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(color_image, f'Distance: {depth_value:.2f}m', (x1, y2 + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(color_image, f'Angle: {angle_deg:.2f} deg', (x1, y2 + 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Optional: reduce the frequency of showing frames to improve performance
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                cv2.imshow('Bag Detection', color_image)

def main():
    rospy.init_node('bag_detection_node')
    
    distance_pub = rospy.Publisher('/bag_distance', Float32, queue_size=10)
    angle_pub = rospy.Publisher('/bag_angle', Float32, queue_size=10)
    rospy.Subscriber('/reach_distance', Bool, reach_distance_callback)

    
    frame_queue = queue.Queue(maxsize=10)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 10)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 10)
    pipeline.start(config)

    model = YOLO("/home/charmander/catkin_ws/src/take_bag/src/bag_detection/best2.pt")
    model.cuda()
    HORIZONTAL_FOV = 65.0
    IMAGE_WIDTH = 640

    capture_thread = threading.Thread(target=capture_frames, args=(pipeline, frame_queue))
    process_thread = threading.Thread(target=process_frames, args=(frame_queue, distance_pub, angle_pub, model, HORIZONTAL_FOV, IMAGE_WIDTH))

    capture_thread.start()
    process_thread.start()

    rospy.spin()  # Keeps the node running

    capture_thread.join()
    process_thread.join()

    pipeline.stop()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
