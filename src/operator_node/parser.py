import numpy
from scipy.interpolate import interp1d

"""
TODO: the following could be abstracted
"""

class Parser:

    # https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.interp1d.html
    interpolate_config = dict(
        kind='cubic',
        bounds_error=False,
        fill_value='extrapolate'
    )


class ParseInterfaceLog(Parser):


    def __init__(self, msg):
        data = numpy.array(msg.data).reshape(3, len(msg.data)//3, order='F')
        self.t = data[0, :].flatten()
        self.h = data[1:, :]
        self.h_fun = interp1d(self.t, self.h, **self.interpolate_config)


    def interpolate(self, t):
        return self.h_fun(t)


    def t_now(self):
        return self.t[-1]


    def h_now(self):
        return self.h[:, -1]


class ParseMousePath(Parser):


    def __init__(self, msg):

        # Extract data
        t = []
        path = []
        for p in msg.poses:
            t.append(p.header.stamp.to_sec())
            path.append([p.position.x, p.position.y])

        # Parse as numpy arrays
        self.t = numpy.array(t)
        self.path = numpy.array(path).T  # 2-by-n

        # Interpolate
        self.path_fun = interp1d(self.t, self.path, **self.interpolate_config)


    def interpolate(self, t):
        return self.path_fun(t)


    def t_now(self):
        return self.t[-1]


    def path_now(self):
        return self.path[:, -1]
