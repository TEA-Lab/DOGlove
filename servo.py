import socket
import struct
import threading
import time

import numpy as np

from dynamixel_sdk import *  # Uses Dynamixel SDK library

#********* DYNAMIXEL Model definition *********
# Control table address
ADDR_TORQUE_ENABLE          = 64
ADDR_LED_RED                = 65
LEN_LED_RED                 = 1         # Data Byte Length
ADDR_GOAL_POSITION          = 116
LEN_GOAL_POSITION           = 4         # Data Byte Length
ADDR_PRESENT_POSITION       = 132
LEN_PRESENT_POSITION        = 4         # Data Byte Length
DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
BAUDRATE                    = 3000000

# DYNAMIXEL Protocol Version 2.0
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0

# Make sure that each DYNAMIXEL ID should have unique ID.
DXL_ID                     = [0, 1, 2, 3, 4]                 # Dynamixe ID

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyUSB0'
# DEVICENAME                  = '/dev/tty.usbserial-FT88YRMM'

# DEFAULT_POS_SCALE = 2.0 * np.pi / 4096  # 0.088 degrees per unit
DEFAULT_POS_SCALE = 2.0 * 180 / 4096  # 0.088 degrees per unit
#********* DYNAMIXEL Model definition *********

data_length = 5  # Number of data values in the block

# Configure UDP settings
udp_ip = "127.0.0.1"  # Localhost IP
udp_port_servo = 5010  # Port to send data to

class ServoReader:
    def __init__(self):
        self.running = True
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Open the port
        if self.portHandler.openPort():
            print("Succeeded to open the port!")
        else:
            print("Failed to open the port!")
            self.stop()
            return
        
        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print(f"Succeeded to change the baudrate to {BAUDRATE}")
        else:
            print(f"Failed to change the baudrate to {BAUDRATE}")
            self.stop()
            return
        
        # Initialize the GroupBulkWrite instance
        self.groupBulkWrite = GroupBulkWrite(self.portHandler, self.packetHandler)
        # Initialize the GroupBulkRead instance
        self.groupBulkRead = GroupBulkRead(self.portHandler, self.packetHandler)

    def read_from_uart(self):
        while self.running:
            # print("Reading from UART...")
            self.process_buffer()

    def process_buffer(self):
        self.process_block()

    def process_block(self):
        # Bulkread present position and LED status
        dxl_comm_result = self.groupBulkRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        
        servo_joint_angles = [180.0, 180.0, 180.0, 180.0, 180.0]
        for id in DXL_ID:
            # Check if groupbulkread data of Dynamixel_id is available
            dxl_getdata_result = self.groupBulkRead.isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupBulkRead getdata failed" % id)
            else:
                # Get present position value
                dxl_present_position = self.groupBulkRead.getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                # print("[ID:%03d] Present Position: %f" % (id, dxl_present_position*DEFAULT_POS_SCALE))
                servo_joint_angles[id] = dxl_present_position*DEFAULT_POS_SCALE
        
        print(f"Servo joint angles: {servo_joint_angles}")

        # Send the first voltage value via UDP
        # message = struct.pack("f", voltages[0])
        format_string = "f" * len(servo_joint_angles)
        # Pack all the values in the list
        message = struct.pack(format_string, *servo_joint_angles)
        self.udp_socket.sendto(message, (udp_ip, udp_port_servo))

    def start(self):
        print("Servo reader started")
        self.thread = threading.Thread(target=self.read_from_uart)
        self.thread.start()

        # Add parameter storage for Dynamixel present position and LED status
        for id in DXL_ID:
            dxl_addparam_result = self.groupBulkRead.addParam(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            if dxl_addparam_result != True:
                print("[ID:%03d] groupBulkRead addparam failed" % id)
                self.stop()
                return

    def stop(self):
        print("Closing UART port...")
        self.running = False
        self.thread.join()

        self.groupBulkRead.clearParam()
        self.groupBulkWrite.clearParam()
        self.portHandler.closePort()
        
        self.udp_socket.close()

if __name__ == "__main__":
    reader = ServoReader()
    try:
        reader.start()
        print("Servo port opened successfully")

        # Keep the main thread alive
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("Program interrupted by user")

    finally:
        reader.stop()
