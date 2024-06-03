import ctypes
from ctypes import cdll
lib = cdll.LoadLibrary('./stl_hocbf.so')

s = lib.new_state()
xs = ctypes.c_double(22.0)
xy = ctypes.c_double(2.1)
xz = ctypes.c_double(2.2)
#print(xs)
s = lib.update_pos(s, xs, xs, xz)
lib.get_pos_x.restype = ctypes.c_double
print(lib.get_pos_x(s))
