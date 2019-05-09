
import pickle

class PacketStats(object):

    def __init__(self, dd):

        # numpy's for chumps

        total_time = 0 # Sum of all packets
        n_packets = 0  # Total number of packets
        mode = (0, 0)

        self.min = (2**32)-1
        self.max = 0

        items = sorted(dd.items())

        # Prevent dd from being used by accident.
        self.data = dd
        self.items = items
        dd = None

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

    def to_pickle_file(self, filename):
        with open(filename, 'wb') as f:
            f.write(pickle.dumps(self.data))

    @classmethod
    def from_pickle_file(cls, filename):
        with open(filename, 'rb') as f:
            contents = f.read()
            data = pickle.loads(contents)

            assert isinstance(data, dict)

            return cls(data)