import numpy
from scipy.interpolate import interp1d


class ParseInterfaceLog:


    # https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.interp1d.html
    interpolate_config = dict(
        kind='cubic',
        bounds_error=False,
        fill_value='extrapolate'
    )


    def __init__(self, msg):
        data = numpy.array(msg.data).reshape(3, len(msg.data)//3, order='F')
        self.t = data[0, :].flatten()
        self.h = data[1:, :]
        self.h_fun = interp1d(self.t, self.h, **interpolate_config)


    def interpolate(self, t):
        return self.h_fun(t)


    def t_now(self):
        return self.t[-1]


    def h_now(self):
        return self.h[:, -1]
