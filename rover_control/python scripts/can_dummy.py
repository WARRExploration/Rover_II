'''
Dummy python script that binds to an already existing serial socketCAN interface
and listens to any data on the CAN bus.
If a SET_SPEED_CMD is received, an entry is added to a dictionary (vel) with the
CAN ID as key and the first 4 received data bytes as a corresponding array.
When a GET_SPEED_CMD is received, the dictionary is looked up for the CAN_ID
and if found the corresponding values are written to the CAN BUS with master_id
as CAN ID
'''

import socket
import struct
import sys

SET_SPEED_CMD = 2
GET_SPEED_CMD = 3

def main():
    if (len(sys.argv) != 2):
        print("Incorrect number of arguments. You need to pass the name of the serial socketCAN interface, e.g. slcan0.")
        return

    fmt = "<IB3x8s"
    master_id = 0x123
    vel = {}
    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    interface = sys.argv[1]

    try:
        sock.bind((interface,))
    except OSError:
        sys.stderr.write("Could not bind to interface '%s'\n" % interface)
        sys.exit()

    while(True):
        try:
            # Receive packets
            can_pkt = sock.recv(16)
            can_id, length, data = struct.unpack(fmt, can_pkt)
            can_id &= socket.CAN_EFF_MASK
            if length == 0:
                continue
            print ("RX - ID: ", can_id, "Data: ", data)
            #print ("Length: ", length)
            data = data[:length]
            if data[0] == SET_SPEED_CMD and length >= 5:
                # Set speed command
                #print ("Speed set!")
                vel[can_id] = data[1:5]


            elif data[0] == GET_SPEED_CMD:
                # Get speed command
                if can_id in vel:
                    resp = bytes([2, vel[can_id][0], vel[can_id][1], vel[can_id][2], vel[can_id][3]])
                else:
                    resp = bytes([2, 0, 0, 0, 0])

                can_pkt = struct.pack(fmt, master_id, len(resp), resp)
                sock.send(can_pkt)
                print("TX - ID: ", master_id, "Data: ", resp)

        except KeyboardInterrupt:
            sys.exit()

if __name__ == "__main__":
    main()
