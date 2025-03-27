import serial
import time
import glob
import os

def find_serial_ports():
    """Lists serial port names
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

if not available_ports:
    print("No serial ports found. Please check connections and permissions.")
    exit()

# Try to connect to each available port and test communication
for port in available_ports:
    try:
        print(f"\nTesting port: {port}")
        ser = serial.Serial(port, 9600, timeout=2)
        
        # Wait for ESP32 to reset after opening serial connection
        time.sleep(2)
        
        # Flush any existing data
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # Test commands
        test_commands = ['R', 'L', 'R', 'L']
        
        for cmd in test_commands:
            print(f"Sending: {cmd}")
            ser.write(cmd.encode())
            
            # Wait for response
            time.sleep(0.5)
            
            # Read any response (with timeout)
            if ser.in_waiting:
                response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                print(f"Response: {response}")
            else:
                print("No response received")
            
            time.sleep(1)
        
        ser.close()
        print(f"Test completed for {port}")
        
    except Exception as e:
        print(f"Error testing {port}: {e}")

print("\nTesting completed for all ports.")
