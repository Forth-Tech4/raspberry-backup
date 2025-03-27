import cv2
import serial
import time
import os
import glob

# Function to find available serial ports
def find_serial_ports():
    """ Lists serial port names
        :returns: A list of the serial ports available on the system
    """
    if os.name == 'nt':  # Windows
        ports = ['COM%s' % (i + 1) for i in range(256)]
    else:  # Linux/macOS
        ports = glob.glob('/dev/tty[A-Za-z]*')
        # Add USB serial device patterns
        ports.extend(glob.glob('/dev/ttyUSB*'))
        ports.extend(glob.glob('/dev/ttyACM*'))
    
    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    
    return result

# Find available serial ports
available_ports = find_serial_ports()
print(f"Available serial ports: {available_ports}")

# Configure the serial connection to ESP32
ser = None
prev_command = ""  # Track the previous command to avoid repeated sends

# Specifically look for ttyUSB0 for ESP32
target_port = '/dev/ttyUSB0'
if target_port in available_ports:
    try:
        # Set higher timeout to avoid issues
        ser = serial.Serial(target_port, 9600, timeout=1)
        print(f"Connected to ESP32 on {available_ports[0]}")
        time.sleep(2)  # Give the connection a second to settle
        
        # Clear any pending data
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # Send a test message to verify connection
        ser.write(b'TEST\n')
        ser.flush()
        time.sleep(0.5)
        
        if ser.in_waiting:
            response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            print(f"ESP32 initialization response: {response}")
        else:
            print("No response from ESP32 during initialization (this is often normal)")
            
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        ser = None

# Load the pre-trained face detection model
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Try to connect to the Lenovo webcam detected in the system
webcam_connected = False

# Try the specific device paths for the Lenovo webcam
for device in ['/dev/video2', '/dev/video3']:
    print(f"Trying Lenovo webcam at {device}...")
    # Try opening the camera using the device path
    cap = cv2.VideoCapture(device)
    
    if cap.isOpened():
        # Try reading a frame to confirm it works
        ret, frame = cap.read()
        if ret:
            frame_height, frame_width = frame.shape[:2]
            center_x = frame_width // 2
            print(f"Success! Lenovo webcam connected at {device}")
            print(f"Frame dimensions: {frame_width}x{frame_height}, Center: {center_x}")
            webcam_connected = True
            break
        else:
            print(f"Camera {device} opened but couldn't read frame.")
            cap.release()
    else:
        print(f"Couldn't open camera {device}.")

# If specific paths didn't work, try standard indices as fallback
if not webcam_connected:
    for camera_idx in range(4):  # Try indices 0, 1, 2, 3
        print(f"Trying camera index {camera_idx}...")
        cap = cv2.VideoCapture(camera_idx)
        
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                frame_height, frame_width = frame.shape[:2]
                center_x = frame_width // 2
                print(f"Success! Camera index {camera_idx} connected.")
                print(f"Frame dimensions: {frame_width}x{frame_height}, Center: {center_x}")
                webcam_connected = True
                break
            else:
                print(f"Camera {camera_idx} opened but couldn't read frame.")
                cap.release()
        else:
            print(f"Couldn't open camera {camera_idx}.")

if not webcam_connected:
    print("Error: Could not connect to any webcam. Please check your connections and permissions.")
    print("Troubleshooting tips:")
    print("1. Run 'ls -l /dev/video*' to see available video devices")
    print("2. Check if user has permission: 'sudo usermod -a -G video $USER'")
    print("3. Try 'v4l2-ctl --list-devices' to list available cameras")
    print("4. Make sure the camera is properly connected via USB")
    exit()

# Function to safely send commands to ESP32
def send_command(command):
    global prev_command, ser
    if ser is None:
        print(f"[SIMULATION] Would send: {command}")
        prev_command = command
        return
        
    if command != prev_command:  # Only send if the command has changed
        try:
            # Add explicit newline character and flush the buffer
            ser.write((command + '\n').encode())
            ser.flush()  # Ensure data is sent immediately
            print(f"Sent: {command}")
            prev_command = command
            
            # Wait briefly for the command to be processed
            time.sleep(0.1)
            
            # Check for any response from ESP32
            if ser.in_waiting:
                response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                print(f"ESP32 response: {response}")
                
        except Exception as e:
            print(f"Error sending command: {e}")
            # Try to reconnect
            try:
                ser.close()
                time.sleep(1)
                ser.open()
                print("Serial connection reestablished")
            except Exception as reconnect_error:
                print(f"Failed to reconnect: {reconnect_error}")
                ser = None
                print("Running in simulation mode now")

try:
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        
        if not ret:
            print("Error: Failed to capture image")
            break
            
        # Convert to grayscale for face detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect faces
        faces = face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30)
        )
        
        # Process the largest face if any are detected
        if len(faces) > 0:
            # Find the largest face based on area
            largest_face = max(faces, key=lambda f: f[2] * f[3])
            x, y, w, h = largest_face
            
            # Draw rectangle around the face
            cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
            
            # Calculate center of the face
            face_center_x = x + w//2
            
            # Determine if face is on left or right side
            if face_center_x < center_x:
                command = "L"  # Face is on the left
            else:
                command = "R"  # Face is on the right
                
            # Send command to ESP32
            send_command(command)
            
            # Display command on frame
            cv2.putText(frame, f"Command: {command}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            # No face detected
            cv2.putText(frame, "No face detected", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Display the frame
        try:
            cv2.imshow('Face Tracking', frame)
        except cv2.error as e:
            print(f"Error displaying frame: {e}")
            print("Running in headless mode (no window display)")
            # If we can't display the window, just log face detection results
            if len(faces) > 0:
                print(f"Face detected! Position: {'Left' if face_center_x < center_x else 'Right'}")
            else:
                print("No face detected")
        
        # Break the loop on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
except KeyboardInterrupt:
    print("Interrupted by user")
    
except Exception as e:
    print(f"Error: {str(e)}")
    
finally:
    # Release resources
    cap.release()
    cv2.destroyAllWindows()
    if ser is not None:
        ser.close()
    print("Program ended")
