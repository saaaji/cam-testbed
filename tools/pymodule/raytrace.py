import os
import ctypes

def init_raytrace():
  components = __file__.split(os.sep)

  dll_path = os.sep.join(components[:components.index('tools') + 1])
  dll_path = os.path.join(dll_path, 'lib/libraytrace.so')

  raytrace = ctypes.CDLL(dll_path)

  raytrace.allocate_image_buffer.argtypes = [ctypes.c_int] * 3
  raytrace.allocate_image_buffer.restype = ctypes.POINTER(ctypes.c_ubyte)

  raytrace.free_image_buffer.argtypes = [ctypes.POINTER(ctypes.c_ubyte)]

  return raytrace