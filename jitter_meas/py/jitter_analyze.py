#!/usr/bin/env python3

from sys import argv, exit
import pickle

from pktstats import PacketStats

# Ideal averages:
#  1/1024 = 9765.625 usec

def process_file(filename):

    print('---------------------------------')
    print('Analyzing %s...' % (filename))

    # BIG WARNING: we're just unpickling whatever is in this file.
    # Don't run this on anything that you did not specifically create.
    stats = PacketStats.from_pickle_file(filename)

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
