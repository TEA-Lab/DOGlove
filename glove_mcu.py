import socket
import struct
import threading
import time

import serial

import numpy as np

# Configure your UART port and baud rate
uart_port = "/dev/ttyACM0"  # Main board UART port
# uart_port = "/dev/tty.usbmodem57340031741"  # Main board UART port
baud_rate = 921600  # Main board baud rate

# Define the header you are looking for
header = bytes([0xAA, 0x55, 0x00, 0x00])
data_length = 16  # Number of data values in the block
block_size = 76  # Total size including header, data, and CRC

# Configure UDP settings
udp_ip = "127.0.0.1"  # Localhost IP
udp_port = 5009  # Port for joint data
udp_port_servo = 5010  # Port for servo data
# udp_port_lra = 5012  # Port for LRA data

class UARTReader:
    def __init__(self):
        self.buffer = bytearray()
        self.running = True
        self.serial_port = serial.Serial(uart_port, baud_rate, timeout=1)
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # self.sock.bind((udp_ip, udp_port_lra))
        self.write_fps = 30

    def read_from_uart(self):
        while self.running:
            # print("Reading from UART...")
            data = self.serial_port.read(self.serial_port.in_waiting or 1)
            if data:
                self.buffer.extend(data)
                self.process_buffer()
            else:
                print("No data received from UART")

    def process_buffer(self):
        while len(self.buffer) >= block_size: # Check if the buffer has enough data to process
            # Find the start index of the first header
            start_index = self.buffer.find(header)

            # No header found, clear the buffer if it's longer than the block size
            if start_index == -1:
                print("No header found. Check the alignment of the data.")

                # If no header is found, clear the buffer if it's longer than the block size
                if len(self.buffer) > block_size:
                    print(f"Discarded data: {self.buffer[:block_size].hex()}")
                    self.buffer = self.buffer[-block_size:]
                break

            # Header found at the beginning of the buffer
            elif start_index == 0:
                # Now find the next header
                next_header_index = self.buffer.find(header, len(header))
                if next_header_index == -1:
                    # If no header is found, break and wait for more data
                    # print(f"No complete block found. Waiting for more data...")#: {self.buffer.hex()}")
                    break
                elif (next_header_index - start_index) != block_size:
                    # If the block is corrupted, discard the data
                    print(f"Corrupted block found: {self.buffer[:next_header_index].hex()}")
                    self.buffer = self.buffer[next_header_index:]
                else:
                    # We found a complete block
                    block = self.buffer[start_index:next_header_index]
                    self.process_block(block)

                    # Remove the processed block from the buffer
                    self.buffer = self.buffer[next_header_index:]
            
            # Header found in the middle of the buffer, discard the data before the header
            else:
                print(f"Discarded data: {self.buffer[:start_index].hex()}")
                self.buffer = self.buffer[start_index:]

    def process_block(self, block):
        # Unpack the block (skip the header)
        data = struct.unpack("<I16iI", block[4:]) # uint32_t reference, 16 int32_t values, uint32_t CRC

        # Extract the 16 int32_t values and CRC
        reference_voltage = data[0]
        data_values = data[1:-1]
        crc = data[-1]

        # Convert data_values to voltages
        voltages = [(float(val) * 5.0 / 0x7FFFFF) for val in data_values]

        # Split CRC into checksum and timestamp
        checksum = (crc >> 16) & 0xFFFF
        timestamp = crc & 0xFFFF

        # Calculate the expected checksum (16-bit sum of data_values)
        expected_checksum = sum(data[:-1]) & 0xFFFF

        # Calculate the joint angles
        reference_voltage = reference_voltage/1000000 # Convert to volts
        joint_angles = [val/reference_voltage*360 for val in voltages if reference_voltage != 0]

        print(f"Voltages: {[round(val, 2) for val in voltages]}")
        print(f"Joint Angle: {[round(val, 2) for val in joint_angles]}")
        print(f"Timestamp: {int(timestamp)}")
        print(f"Checksum: {(checksum)} (Expected: {(expected_checksum)})")
        print(f"Checksum Valid: {checksum == expected_checksum}")
        print("-" * 40)

        # Send the first voltage value via UDP
        # message = struct.pack("f", voltages[0])
        format_string = "f" * len(joint_angles)
        # Pack all the values in the list
        message = struct.pack(format_string, *joint_angles)
        self.udp_socket.sendto(message, (udp_ip, udp_port))
    
    # def lra_control(self, channel, wave, duration):
    #     """send_data = 0x55, 0xAA, channel(0-4), wave, duration_h, duration_l, checksum"""
    #     if channel not in range(5):
    #         print("Invalid channel. Please choose a channel between 0-4.")
    #         return
    #     if wave not in range(256):
    #         print("Invalid wave. Please choose a wave between 0-255.")
    #         return
    #     if duration not in range(65536):
    #         print("Invalid duration. Please choose a duration between 0-65535.")
    #         return
    #     duration_H = (duration >> 8) & 0xFF
    #     duration_L = duration & 0xFF
    #     checksum = (channel + wave + duration_H + duration_L) & 0xFF
    #     send_data = [0x55, 0xAA, channel, wave, duration_H, duration_L, checksum]
    #     # print(f"send data: {bytes(send_data).hex()}")
    #     self.serial_port.write(bytes(send_data))
    
    # def listen(self):
    #     print(f"Listening on {udp_ip}:{udp_port_lra}...")
    #     while self.running:
    #         data, addr = self.sock.recvfrom(4*4)  # 4 bytes per float, 16 floats
    #         if data:
    #             # Unpack the received float
    #             received_kp = struct.unpack("f"*4, data)

    #             # Store the most recent pressure
    #             self.most_recent_kp = received_kp

    #             # print(f"Received kp (as float): {kp}")
    #             # print("-" * 40)

    #             # Control the LRA
    #             k = 100
    #             # k = 2000
    #             for i in range(len(received_kp)):
    #                 if received_kp[i] > 10 and received_kp[i] < k:
    #                     self.lra_control(i, 56, int(1000/self.write_fps))
    #                     # self.lra_control(i, 222, int(1000/self.write_fps))
    #                 else:
    #                     self.lra_control(i, 222, 100)
    #             time.sleep(1/self.write_fps)

    def start(self):
        print("UART reader started")
        self.thread = threading.Thread(target=self.read_from_uart)
        self.thread.start()
        # self.thread_lra = threading.Thread(target=self.listen)
        # self.thread_lra.start()

    def stop(self):
        print("Closing UART port...")
        self.running = False
        self.thread.join()
        # self.thread_lra.join()
        self.udp_socket.close()
        # self.sock.close()
        self.serial_port.close()
        

class UDPReceiver:
    def __init__(self):
        self.most_recent_joints = None
        self.most_recent_servo = None
        self.urdf_joints = None
        self.running = True
        self.sock1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock1.bind((udp_ip, udp_port))
        self.sock2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock2.bind((udp_ip, udp_port_servo))
        self.length = data_length

    def listen1(self):
        print(f"Listening on {udp_ip}:{udp_port}...")
        while self.running:
            data, addr = self.sock1.recvfrom(4*self.length)  # 4 bytes per float, 16 floats
            if data:
                # Unpack the received float
                received_joints = struct.unpack("f"*self.length, data)

                # Store the most recent joint angle
                if received_joints is not None:
                    self.most_recent_joints = received_joints

                    # print(f"Received joint angle (as float): {received_joints}")
                    # print(f"Converted joint angle (as int): {self.most_recent_joints}")
                    # print("-" * 40)
            
    def listen2(self):
        print(f"Listening on {udp_ip}:{udp_port_servo}...")
        while self.running:     
            data, addr = self.sock2.recvfrom(4*5)  # 4 bytes per float, 5 floats
            if data:
                # Unpack the received float
                received_servo = struct.unpack("f"*5, data)

                # Store the most recent servo angle
                if received_servo is not None:
                    self.most_recent_servo = received_servo

                    # print(f"Received servo angle (as float): {received_servo}")
                    # print(f"Converted servo angle (as int): {self.most_recent_servo}")
                    # print("-" * 40)

    def start(self):
        self.thread1 = threading.Thread(target=self.listen1)
        self.thread1.start()

        self.thread2 = threading.Thread(target=self.listen2)
        self.thread2.start()

    def stop(self):
        self.running = False
        self.thread1.join()
        self.thread2.join()
        
        self.sock1.close()
        self.sock2.close()

    def convert_to_urdf_joint(self):
        # Convert the joint angle to URDF joint angle
        joint_angles = self.most_recent_joints
        servo_angles = self.most_recent_servo

        if joint_angles is not None and servo_angles is not None:
            urdf_joint_angles = [0.0] * 21

            urdf_joint_angles[0] = np.deg2rad(-(joint_angles[0]-90)) # 0 - thumb_dip
            urdf_joint_angles[1] = np.deg2rad(joint_angles[1]-270) # 1 - thumb_pip
            urdf_joint_angles[2] = np.deg2rad(joint_angles[2]-180) # 2 - thumb_mcp_s
            urdf_joint_angles[3] = np.deg2rad(joint_angles[3]-270) # 3 - thumb_mcp_r

            urdf_joint_angles[4] = np.deg2rad(-(joint_angles[4]-90)) # 4 - index_dip
            urdf_joint_angles[5] = np.deg2rad(joint_angles[5]-270) # 5 - index_pip
            urdf_joint_angles[6] = np.deg2rad(-(joint_angles[6]-180)) # 6 - index_mcp_s

            urdf_joint_angles[7] = np.deg2rad(-(joint_angles[7]-90)) # 7 - middle_dip
            urdf_joint_angles[8] = np.deg2rad(joint_angles[8]-270) # 8 - middle_pip
            urdf_joint_angles[9] = np.deg2rad(-(joint_angles[9]-180)) # 9 - middle_mcp_s

            urdf_joint_angles[10] = np.deg2rad(-(joint_angles[10]-90)) # 10 - ring_dip
            urdf_joint_angles[11] = np.deg2rad(joint_angles[11]-270) # 11 - ring_pip
            urdf_joint_angles[12] = np.deg2rad(-(joint_angles[12]-180)) # 12 - ring_mcp_s

            urdf_joint_angles[13] = np.deg2rad(-(joint_angles[13]-90)) # 13 - little_dip
            urdf_joint_angles[14] = np.deg2rad(joint_angles[14]-270) # 14 - little_pip
            urdf_joint_angles[15] = np.deg2rad(-(joint_angles[15]-180)) # 15 - little_mcp_s

            urdf_joint_angles[16] = np.deg2rad(-(servo_angles[0]-182.2)) # 16 - thumb_mcp_b
            urdf_joint_angles[17] = np.deg2rad(-(servo_angles[1]-165.2)) # 17 - index_mcp_b
            urdf_joint_angles[18] = np.deg2rad(-(servo_angles[2]-150.29)) # 18 - middle_mcp_b
            urdf_joint_angles[19] = np.deg2rad(-(servo_angles[3]-151.44)) # 19 - ring_mcp_b
            urdf_joint_angles[20] = np.deg2rad(-(servo_angles[4]-145.9)) # 20 - little_mcp_b

            urdf_joint_angles = tuple(urdf_joint_angles)

            self.urdf_joints = urdf_joint_angles


    def get_most_recent_joints(self):
        self.convert_to_urdf_joint()
        #print(self.urdf_joints)
        return self.urdf_joints


if __name__ == "__main__":
    reader = UARTReader()
    try:
        print(f"Opening UART port {uart_port} at {baud_rate} baudrate...")
        reader.start()
        print("UART port opened successfully")

        # Keep the main thread alive
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("Program interrupted by user")

    finally:
        reader.stop()
