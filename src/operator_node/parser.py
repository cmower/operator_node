import numpy


def reconstruct_interface_log_msg(msg, Nd):
    """Reconstruct the interface log from a std_msgs/Float64MultiArray message

Syntax
------

t, h = reconstruct_interface_log_msg(msg)

Input
-----

msg [std_msgs/Float64Array]
  ROS message recieved from an operator_interface_logger node.

Nd [int]
  Number of dimensions for the operator signal.

Output
------

t [numpy.ndarray]
  An N-array of time stamps.

h [numpy.ndarray]
  An Nd-by-N array containing N operator signals.

    """
    Nd1 = Nd+1  # operator signals and time
    data = numpy.array(msg.data).reshape(Nd1, len(msg.data)//(Nd1), order='F')
    t = data[0, :]
    h = data[1:, :]
    return t, h
