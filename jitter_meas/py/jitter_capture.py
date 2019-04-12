#!/usr/bin/env python3

import socket
import struct
from collections import defaultdict
from sys import argv, exit
import pickle

import datetime

UDP_ADDR = '0.0.0.0'
UDP_PORT = 37250

def handle_packet(data):

    num_samples = len(data) // 4
    
    samples = []
    for x in range(num_samples):
        index = x * 4
        val = struct.unpack('<I', data[index:index+4])
        samples.append(val[0])

    return samples

def main():

    # If argument is provided, use that
    if len(argv) == 1:
        # No filename provided. Use one that's reasonable, with invalid chars removed.
        date = datetime.datetime.now().replace(microsecond=0).isoformat().replace(':', '.')
        filename = '%s.pkt' % (date)
    elif len(argv) == 2:
        filename = argv[1]
    else:
        print('Usage: %s [FILENAME]' % (argv[0]))
        exit(1)

    print('Writing packet stats to %s...' % (filename))
    statsfile = open(filename, 'wb')

    # Create a UDP socket.
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    buckets = defaultdict(int)

    # Bind to our local interface.
    BIND_INFO = (UDP_ADDR, UDP_PORT)
    sock.bind(BIND_INFO)

    total = 0
    clean_exit = False

    try:
        while True:
            data, addr = sock.recvfrom(2048)
            samples = handle_packet(data)
            total += len(samples)

            print(('\rRecv: %4d Total: %d' % (len(samples), total)).ljust(40), end='\r')

            for s in samples:
                buckets[s] += 1
            
    except KeyboardInterrupt:
        print('\r\nReceived keyboard interrupt.')
        clean_exit = True
    finally:
        print('%sWriting packet data...' % ('' if clean_exit else '\r\n'))
        data = pickle.dumps(buckets)
        statsfile.write(data)
        statsfile.close()

if __name__ == '__main__':
    main()
