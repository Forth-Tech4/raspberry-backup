import cv2
import time

# Try device 14 with different API backends
backends = [cv2.CAP_V4L2, cv2.CAP_GSTREAMER, cv2.CAP_ANY]
device = 14  # Try each of the devices that opened: 14, 15, 21, 22

for backend in backends:
    print(f"Trying device {device} with backend {backend}...")
    cap = cv2.VideoCapture(device, backend)
    
    # Set lower resolution which might help
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    # Set a longer timeout
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
    if cap.isOpened():
        print(f"Device {device} opened with backend {backend}")
        
        # Try multiple times to read a frame
        for attempt in range(5):
            print(f"Read attempt {attempt+1}...")
            ret, frame = cap.read()
            if ret:
                print("Successfully read a frame!")
                cv2.imwrite(f"success_{device}_{backend}.jpg", frame)
                break
            else:
                print("Failed to read frame, waiting...")
                time.sleep(2)
                
        cap.release()
    else:
        print(f"Failed to open device {device} with backend {backend}")

print("Test complete")
