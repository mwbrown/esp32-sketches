#!/usr/bin/env python3

from sys import argv, exit
import pickle

# Ideal averages:
#  1/1024 = 9765.625 usec

class PacketStats(object):

    def __init__(self, dd):

        # numpy's for chumps

        total_time = 0 # Sum of all packets
        n_packets = 0  # Total number of packets
        mode = (0, 0)

        self.min = (2**32)-1
        self.max = 0

        items = sorted(dd.items())
        dd = None # Prevent dd from being used by accident

        for usec, occurs in items:
            total_time += usec * occurs
            n_packets += occurs

            if self.min > usec:
                self.min = usec

            if self.max < usec:
                self.max = usec

            if occurs > mode[1]:
                mode = (usec, occurs)

        self.mean = float(total_time) / n_packets
        self.mode = mode[0]
        self.midrange = float(self.min + self.max) / 2

        var_sum = 0
        median_count = n_packets / 2

        # Loop through again for anything that requires knowing the mean or popcount.
        for usec, occurs in items:

            # Take the sum of the square of the deviation from the mean
            var_sum = ((usec - self.mean) ** 2) * occurs

            # Are we still counting towards the median?
            if median_count > 0:
                median_count -= occurs

                # Did we exceed the median point?
                if median_count < 0:
                    self.median = usec

        self.variance = float(var_sum) / n_packets
        self.std_dev = self.variance ** 0.5
        self.pop_size = n_packets

def process_file(filename):

    print('---------------------------------')
    print('Analyzing %s...' % (filename))
    with open(filename, 'rb') as f:

        # BIG WARNING: we're just unpickling whatever is in this file.
        # Don't run this on anything that you did not specifically create.
        contents = f.read()
        packet_info = pickle.loads(contents)

        assert isinstance(packet_info, dict)

        stats = PacketStats(packet_info)
        print('pop. size:', stats.pop_size)
        print('min:      ', stats.min)
        print('mean:     ', stats.mean)
        print('max:      ', stats.max)
        print('')
        print('midrange: ', stats.midrange)
        print('mean-min: ', stats.mean - stats.min)
        print('max-mean: ', stats.max - stats.mean)
        print('')
        print('median:   ', stats.median)
        print('mode:     ', stats.mode)
        print('variance: ', stats.variance)
        print('std. dev: ', stats.std_dev)
        print('')
        print('---------------------------------')
        print('')

def main():

    if len(argv) == 1:
        print('Usage: %s FILE1 FILE2 FILE3 ...' % (argv[0]))
        print('See jitter_capture.py to create file(s) for analysis.')
        exit(1)

    for filename in argv[1:]:
        try:
            process_file(filename)
        except Exception as e:
            print('Error processing %s: %s' % (filename, e))


if __name__ == '__main__':
    main()
