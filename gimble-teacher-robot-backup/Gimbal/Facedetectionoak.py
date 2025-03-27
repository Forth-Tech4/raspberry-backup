#!/usr/bin/env python3

import cv2
import depthai as dai
import time
import serial
import math
import argparse
import os
import numpy as np

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--port', type=str, default='/dev/ttyUSB0',
                        help='ESP32 serial port, default is /dev/ttyUSB0')
    parser.add_argument('-b', '--baudrate', type=int, default=115200,
                        help='Serial baudrate, default is 115200')
    parser.add_argument('-d', '--display', action='store_true',
                        help='Display the camera feed')
    return parser.parse_args()

class FaceTrackingGimbal:
    def __init__(self, esp32_port, baudrate=115200, display=False):
        self.display = display
        
        # Set up serial connection to ESP32
        try:
            self.esp32 = serial.Serial(esp32_port, baudrate, timeout=1)
            print(f"Connected to ESP32 on {esp32_port}")
            time.sleep(2)  # Allow time for serial connection to establish
        except Exception as e:
            print(f"Error connecting to ESP32: {e}")
            exit(1)
            
        # Configure parameters for gimbal control
        self.prev_command_time = time.time()
        self.command_interval = 0.1  # Send commands every 100ms
        
        # Camera parameters
        self.frame_width = 640
        self.frame_height = 480
        self.center_x = self.frame_width // 2
        self.center_y = self.frame_height // 2
        
        # Gimbal control parameters
        self.x_deadzone = 50  # Pixels
        self.y_deadzone = 50  # Pixels
        
        # Movement thresholds
        self.min_face_depth = 500   # mm
        self.max_face_depth = 3000  # mm

        # Initialize DepthAI pipeline
        self.initialize_pipeline()
        
    def initialize_pipeline(self):
        # Create pipeline
        self.pipeline = dai.Pipeline()
        
        # Define sources and outputs
        
        # Color camera
        cam_rgb = self.pipeline.create(dai.node.ColorCamera)
        cam_rgb.setPreviewSize(self.frame_width, self.frame_height)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
        
        # Stereo camera for depth
        mono_left = self.pipeline.create(dai.node.MonoCamera)
        mono_right = self.pipeline.create(dai.node.MonoCamera)
        stereo = self.pipeline.create(dai.node.StereoDepth)
        
        # Set resolution for mono cameras
        mono_resolution = dai.MonoCameraProperties.SensorResolution.THE_400_P
        mono_left.setResolution(mono_resolution)
        mono_right.setResolution(mono_resolution)
        # Use updated API for camera sockets
        mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)  # Left camera
        mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)  # Right camera
        
        # StereoDepth configuration
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)  # RGB camera
        
        # Create outputs
        xout_rgb = self.pipeline.create(dai.node.XLinkOut)
        xout_depth = self.pipeline.create(dai.node.XLinkOut)
        
        xout_rgb.setStreamName("rgb")
        xout_depth.setStreamName("depth")
        
        # Linking
        cam_rgb.preview.link(xout_rgb.input)
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)
        stereo.depth.link(xout_depth.input)
        
        print("Pipeline initialized")
        
    def send_command(self, command):
        """Send command to ESP32"""
        try:
            self.esp32.write(f"{command}\n".encode())
            print(f"Sent command: {command}")
        except Exception as e:
            print(f"Error sending command: {e}")
    
    def calculate_gimbal_commands(self, face_center, depth):
        """Calculate gimbal movement commands based on face position and depth"""
        if face_center is None:
            return None
            
        face_x, face_y = face_center
        
        # Calculate offsets from center
        x_offset = face_x - self.center_x
        y_offset = face_y - self.center_y
        
        # Determine commands based on offset
        commands = []
        
        # X-axis (Left/Right)
        if abs(x_offset) > self.x_deadzone:
            # Convert pixel offset to angle
            angle = int(abs(x_offset) / 3)  # Simple conversion for demonstration
            angle = min(angle, 1000)  # Limit to maximum value
            
            if x_offset < 0:
                commands.append(f"L {angle}")
            else:
                commands.append(f"R {angle}")
        
        # Y-axis (Up/Down)
        if abs(y_offset) > self.y_deadzone:
            # Convert pixel offset to speed between 1000-2000 range
            speed = 1000 + int((abs(y_offset) / (self.frame_height / 2)) * 1000)
            speed = max(1000, min(speed, 2000))  # Constrain to 1000-2000 range
            
            if y_offset < 0:
                commands.append(f"U {speed}")
            else:
                commands.append(f"D {speed}")
        
        return commands
    
    def get_depth_at_point(self, depth_frame, point):
        """Get the depth at a specific point"""
        if point is None or depth_frame is None:
            return None
            
        x, y = point
        
        # Ensure within bounds
        if (0 <= x < depth_frame.shape[1] and 
            0 <= y < depth_frame.shape[0]):
            # Get depth value at point
            depth_value = depth_frame[y, x]
            return depth_value
            
        return None
    
    def detect_faces(self, frame):
        """Detect faces using OpenCV's built-in face detector"""
        # Convert to grayscale for face detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Load the pre-trained face detector (Haar Cascade)
        face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        
        # Detect faces
        faces = face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30),
            flags=cv2.CASCADE_SCALE_IMAGE
        )
        
        if len(faces) > 0:
            # Get the largest face
            largest_face = sorted(faces, key=lambda x: x[2] * x[3], reverse=True)[0]
            x, y, w, h = largest_face
            
            # Return face bounding box and center
            face_center = (x + w//2, y + h//2)
            return (x, y, x+w, y+h), face_center
            
        return None, None
    
    def run(self):
        """Main loop for face tracking and gimbal control"""
        try:
            # Connect to device and start pipeline
            with dai.Device(self.pipeline) as device:
                print("Connected to OAK-D Lite")
                
                # Get output queues
                q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
                q_depth = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
                
                face_center = None
                
                print("Starting face tracking...")
                
                while True:
                    # Get camera frames
                    in_rgb = q_rgb.tryGet()
                    if in_rgb is None:
                        continue
                        
                    in_depth = q_depth.tryGet()
                    
                    # Get RGB frame for display
                    frame = in_rgb.getCvFrame()
                    
                    # Get depth frame if available
                    depth_frame = None
                    depth_frame_view = None
                    if in_depth is not None:
                        depth_frame = in_depth.getFrame()
                        # Convert depth frame to normalized 8-bit for visualization
                        depth_frame_view = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
                    
                    # Detect faces using OpenCV
                    face_bbox, face_center = self.detect_faces(frame)
                    
                    # If a face is detected
                    if face_bbox is not None and face_center is not None:
                        x1, y1, x2, y2 = face_bbox
                        
                        # Draw bounding box on frame
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        
                        # Get depth at face if depth frame is available
                        depth = None
                        if depth_frame is not None:
                            depth = self.get_depth_at_point(depth_frame, face_center)
                            if depth is not None:
                                cv2.putText(frame, f"Depth: {depth} mm", (x1, y1 - 10),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        
                        # Calculate and send commands to gimbal
                        current_time = time.time()
                        if current_time - self.prev_command_time >= self.command_interval:
                            commands = self.calculate_gimbal_commands(face_center, depth)
                            if commands:
                                for cmd in commands:
                                    self.send_command(cmd)
                            self.prev_command_time = current_time
                    
                    # Display frames if enabled
                    if self.display:
                        cv2.imshow("RGB", frame)
                        if depth_frame_view is not None:
                            cv2.imshow("Depth", depth_frame_view)
                        
                        # Exit on 'q' press
                        if cv2.waitKey(1) == ord('q'):
                            break
        except Exception as e:
            print(f"Error during execution: {e}")
            import traceback
            traceback.print_exc()
                            
        # Clean up
        if self.display:
            cv2.destroyAllWindows()
        self.esp32.close()
        print("Face tracking stopped")

if __name__ == "__main__":
    # Parse command line arguments
    args = parse_args()
    
    # Create and run the face tracking gimbal
    tracker = FaceTrackingGimbal(
        esp32_port=args.port,
        baudrate=args.baudrate,
        display=args.display
    )
    
    try:
        tracker.run()
    except KeyboardInterrupt:
        print("Program interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
