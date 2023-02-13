import socket

class MAVSend(object):

    udp_addr = ('<broadcast>', 9999)

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET,socket.SO_BROADCAST,1)

    def write(self, buf):
        send_callback = self.udp_socket.sendto(buf, self.udp_addr)
        # print('send')