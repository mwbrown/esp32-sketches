#!/usr/bin/env python3

from sys import argv, exit
from pktstats import PacketStats
import pickle

REMOVAL_THRESHOLD = 5000 # standard deviations to tolerate before removal

def process_file(filename):

    outfile = '%s.trim' % (filename)

    print('---------------------------------')
    print('Analyzing %s...' % (filename))

    # BIG WARNING: we're just unpickling whatever is in this file.
    # Don't run this on anything that you did not specifically create.
    packet_info = PacketStats.from_pickle_file(filename)

    print(packet_info.items)

    removals = []
    for k,v in packet_info.data.items():

        dev = abs(k - packet_info.mean) / packet_info.std_dev

        if dev >= REMOVAL_THRESHOLD:
            print('{} is outlier'.format(k))
            removals.append(k)

    for k in removals:
        del packet_info.data[k]

    with open(outfile, 'wb') as f:
        f.write(pickle.dumps(packet_info.data))

def main():

    if len(argv) != 2:
        print('Usage: %s FILE ...' % (argv[0]))
        print('See jitter_capture.py to create file(s) for analysis.')
        exit(1)

    try:
        process_file(argv[1])
    except Exception as e:
        print('Error processing %s: %s' % (argv[1], e))


if __name__ == '__main__':
    main()
