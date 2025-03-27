#!/usr/bin/env python3

import serial
import time
import random

# Serial port configuration - adjust if needed
SERIAL_PORT = '/dev/ttyUSB0'  # Change to match your ESP32 port
BAUD_RATE = 115200

# Maximum motor rotation time (milliseconds)
MAX_MOTOR_TIME = 3000

# Commands for the gimbal
COMMANDS = {
    'UP': 'U',        # Up - DC motor forward
    'DOWN': 'D',      # Down - DC motor reverse
    'RIGHT': 'R',     # Right - Stepper clockwise
    'LEFT': 'L',      # Left - Stepper counter-clockwise
    'LASER_ON': 'A1', # Turn laser on
    'LASER_OFF': 'A0',# Turn laser off
    'LIGHT_ON': 'B1', # Turn light on
    'LIGHT_OFF': 'B0',# Turn light off
    'FACE': 'F1'      # Trigger face detected tone
}

def setup_serial():
    """Initialize serial connection to ESP32"""
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud")
        time.sleep(2)  # Allow time for connection to establish
        return ser
    except Exception as e:
        print(f"Error connecting to serial port: {e}")
        exit(1)

def send_command(ser, command, value=None):
    """Send command to ESP32 with optional value"""
    if value is not None:
        cmd = f"{command} {value}\n"
    else:
        cmd = f"{command}\n"
    
    print(f"Sending: {cmd.strip()}")
    ser.write(cmd.encode())
    
    # If it's a motor command, return the time needed to wait
    if command in ['U', 'D', 'R', 'L']:
        # Add 30ms buffer to the time value
        return value + 30
    return 100  # Default wait time for non-motor commands

def generate_random_motor_command():
    """Generate a random motor command"""
    # Choose random direction
    direction = random.choice(['U', 'D', 'R', 'L'])
    
    # Generate random time (between 200ms and MAX_MOTOR_TIME)
    time_ms = random.randint(200, MAX_MOTOR_TIME)
    
    return direction, time_ms

def generate_random_light_command():
    """Generate a random light or laser command"""
    # More variation in light/laser commands
    options = [
        ('A', 1),  # Laser on
        ('A', 0),  # Laser off
        ('B', 1),  # Light on
        ('B', 0),  # Light off
        ('F', 1)   # Face detection sound
    ]
    command, value = random.choice(options)
    return command, value

def main():
    # Setup serial connection
    ser = setup_serial()
    
    try:
        print("Starting random command sequence...")
        print("Press Ctrl+C to stop")
        
        # Run a sequence of random commands
        while True:
            # Randomly decide to send a motor or light/laser command
            if random.random() < 0.5:  # 50% chance for motor command
                command, value = generate_random_motor_command()
                # Send the motor command
                wait_time = send_command(ser, command, value)
                
                # Randomly turn on laser or light during motor movement (more often)
                if random.random() < 0.7:
                    light_cmd = random.choice(['A1', 'B1'])
                    send_command(ser, light_cmd[0], int(light_cmd[1]))
            else:  # 50% chance for light/laser command
                command, value = generate_random_light_command()
                # Send the light/laser command
                wait_time = send_command(ser, command, value)
            
            # Wait for the command to complete before sending next
            time.sleep(wait_time / 1000.0)  # Convert ms to seconds
            
            # Random pause between commands (100-300ms)
            pause = random.randint(100, 300) / 1000.0
            time.sleep(pause)
            
    except KeyboardInterrupt:
        print("\nStopping command sequence")
    finally:
        # Make sure to stop any movement before exiting
        send_command(ser, 'U', 0)  # Sending 0 time should stop motors
        send_command(ser, 'D', 0)
        send_command(ser, 'A', 0)  # Turn off laser
        send_command(ser, 'B', 0)  # Turn off light
        
        # Close serial connection
        if ser.is_open:
            ser.close()
            print("Serial connection closed")

if __name__ == "__main__":
    main()
