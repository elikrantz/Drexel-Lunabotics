import serial
import time

# Initialize the serial connection
serial_device = '/dev/serial/by-path/pci-0000:06:00.4-usb-0:2:1.0'  # Change to appropriate serial port
ser = serial.Serial(serial_device, 9600, timeout=2)  # 9600 Baud rate

print("We are in")
time.sleep(2)  # Wait for serial connection to initialize

for i in range(3):
    print(i)
    ser.reset_input_buffer()  # Equivalent to FlushIOBuffers()
    
    ser.write(b'S')  # Send the command (b'S' sends a byte string)
    time.sleep(1)
    
    if ser.in_waiting > 0:  # If data is available
        received = ser.readline().decode('utf-8').strip()  # Reading the response
        print(received)
    
    time.sleep(2)

ser.close()
print("We are out")
time.sleep(1)
