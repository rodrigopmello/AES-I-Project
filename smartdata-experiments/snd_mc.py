import socket
import struct
import sys

multicast_group = '224.0.0.1'
server_address = ('', 5001)

# Create the socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


sock.bind(server_address)

# Tell the operating system to add the socket to the multicast group
# on all interfaces.
group = socket.inet_aton(multicast_group)
mreq = struct.pack('4sL', group, socket.INADDR_ANY)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

# Receive/respond loop
addresses = []
print(sys.stderr, '\nwaiting to receive message')
while True:
    
    data, address = sock.recvfrom(2048)
    
    print(sys.stderr, 'received %s bytes from %s' % (len(data), address))
    print(sys.stderr, data)
    addresses.append(address)

    for addr in addresses:
        if addr[1] != address[1]:    
            print(sys.stderr, 'sending message to others ', addr)
            sock.sendto(data, addr)
