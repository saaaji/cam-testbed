import os
import ctypes

class Camera(ctypes.Structure):
  _fields_ = [
    ("resolution", ctypes.c_int * 2),
    ("sparse_intrinsics", ctypes.c_float * 4),
    ("distortion_coeffs", ctypes.c_float * 8),
  ]

def init_raytrace():
  components = __file__.split(os.sep)

  dll_path = os.sep.join(components[:components.index('tools') + 1])
  dll_path = os.path.join(dll_path, 'lib/libraytrace.so')

  raytrace = ctypes.CDLL(dll_path)

  # init_workers
  # raytrace.init_workers.argtypes = ctypes.c_int

  # allocate_image_buffer
  raytrace.allocate_image_buffer.argtypes = [ctypes.c_int] * 3
  raytrace.allocate_image_buffer.restype = ctypes.POINTER(ctypes.c_ubyte)

  # free_image_buffer
  raytrace.free_image_buffer.argtypes = [ctypes.POINTER(ctypes.c_ubyte)]

  # render_frame
  raytrace.render_frame.argtypes = [ctypes.POINTER(ctypes.c_ubyte), ctypes.POINTER(Camera), ctypes.c_float]

  return raytrace