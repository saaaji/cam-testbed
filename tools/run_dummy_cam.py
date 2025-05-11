import sys
import signal
import subprocess
import glob
import os
import argparse
import cv2
import time
import math
import re
import numpy as np
import ctypes

DEFAULT_VENDOR = 'DummyVendor'
DEFAULT_PRODUCT_NAME = 'DummyV4L2Camera'
DEFAULT_SERIAL_NO = 'DUMMY_SERIAL_NO'
DEFAULT_SIZE = (1280, 720)
DEFAULT_FPS = '60'

SIZE_RE = r'^(?P<width>\d+)x(?P<height>\d+)\Z'

CLEANUP_TASKS = []
INIT_CMDS = [
  'sudo modprobe -r v4l2loopback',
  'sudo modprobe v4l2loopback devices=1 exclusive_caps=1',
]

def init_raytrace():
  dll_path = os.path.split(__file__)
  dll_path = os.path.normpath(dll_path[0])
  dll_path = os.path.join(dll_path, 'lib/libraytrace.so')
  
  raytrace = ctypes.CDLL(dll_path)

  raytrace.allocate_image_buffer.argtypes = [ctypes.c_int] * 3
  raytrace.allocate_image_buffer.restype = ctypes.POINTER(ctypes.c_ubyte)

  raytrace.free_image_buffer.argtypes = [ctypes.POINTER(ctypes.c_ubyte)]

  return raytrace

def get_active_pid_cmd(dev_path):
  try:
    res = subprocess.run(
      ['lsof', dev_path],
      capture_output=True,
      check=True,
      text=True,
    )

    lines = res.stdout.strip().split('\n')
    users = []
    for line in lines[1:]:
      parts = line.split()
      if len(parts) >= 2:
        users.append((int(parts[1]), parts[0]))
    return users
  except subprocess.CalledProcessError:
    return []

def init(args):
  # reset the module & create loopback device
  for cmd in INIT_CMDS:
    try:
      subprocess.run(cmd.split(), capture_output=True, check=True)
    except subprocess.CalledProcessError as e:
      print(f'init command failed (exit code {e.returncode})')

  # find the driver
  names = glob.glob('/sys/devices/virtual/video4linux/video*')
  assert len(names) == 1

  [dev_name] = names
  _, dev_path = os.path.split(dev_name)
  dev_path = os.path.join('/dev', dev_path)

  assert os.path.exists(dev_path)

  # create symlink (template: usb-<vendor>_<product>_<serial>-video-index<index>)
  link_name = f'usb-{args.vendor}_{args.product}_{args.serial}-video-index0'
  link_path = os.path.join('/dev/v4l/by-id', link_name)
  os.symlink(dev_path, link_path)

  # kill all programs using this camera on shutdown to ensure proper cleanup
  # (including ffmpeg)
  def kill_deps():
    users = get_active_pid_cmd(dev_path)

    for pid, cmd in set(users):
      print(f'killing dependency: {cmd} (PID {pid})')
      os.kill(pid, signal.SIGKILL)

  CLEANUP_TASKS.append(lambda: os.remove(link_path) if os.path.islink(link_path) else None)
  CLEANUP_TASKS.append(kill_deps)
  CLEANUP_TASKS.append(lambda: subprocess.run('sudo modprobe -r v4l2loopback'.split(), capture_output=True, check=True))

  return dev_path, link_path

def cleanup():
  [task() for task in CLEANUP_TASKS]

def signal_handler(sig, frame):
  cleanup()
  sys.exit(0)

def run(args):
  dev_path, link_path = init(args)
  print(f'creating dummy camera at {dev_path}')
  print(f'|__ (symlink) {link_path}')

  # stream size configuration
  width = DEFAULT_SIZE[0]
  height = DEFAULT_SIZE[1]
  fps = int(args.fps)

  if (search_res := re.search(SIZE_RE, args.dim or '')) is not None:
    groupdict = search_res.groupdict()
    width = int(groupdict.width)
    height = int(groupdict.height)

  # launch ffmpeg to receive and write frames to loopback device
  canvas = np.zeros((width, height, 3), dtype=np.uint8)
  proc = subprocess.Popen(
    f'ffmpeg -f rawvideo -pix_fmt bgr24 -s {width}x{height} -i - -f v4l2 -pix_fmt yuyv422 {dev_path}'.split(),
    stdin=subprocess.PIPE,
    stdout=subprocess.DEVNULL,
    stderr=subprocess.STDOUT,
  )

  def kill_ffmpeg():
    if proc.poll() is None:
      proc.stdin.close()
      proc.terminate()

  CLEANUP_TASKS.append(kill_ffmpeg)

  # initialize image buffer
  raytrace = init_raytrace()

  buffer_ptr = raytrace.allocate_image_buffer(width, height, 3)
  buffer_size = width * height * 3
  CLEANUP_TASKS.append(lambda: raytrace.free_image_buffer(buffer_ptr))
  
  while True:
    start = time.time()
    
    # image generation
    # canvas = np.random.randint(0, 256, size=(width, height, 3), dtype=np.uint8)
    # proc.stdin.write(canvas.tobytes())

    proc.stdin.write(ctypes.string_at(buffer_ptr, buffer_size))
    
    # framerate regulation
    end = time.time()
    time.sleep(max(0, (1 / fps) - (end - start)))

  cleanup()

def main():
  # configure command line arguments
  parser = argparse.ArgumentParser(
    prog='dummy_cam',
    description='run dummy v4l2 camera',
  )

  parser.add_argument('-s', '--serial', required=False, default=DEFAULT_SERIAL_NO)
  parser.add_argument('-v', '--vendor', required=False, default=DEFAULT_VENDOR)
  parser.add_argument('-p', '--product', required=False, default=DEFAULT_PRODUCT_NAME)
  parser.add_argument('-d', '--dim', required=False)
  parser.add_argument('-f', '--fps', required=False, default=DEFAULT_FPS)
  args = parser.parse_args()

  # cleanup handlers
  signal.signal(signal.SIGINT, signal_handler)
  signal.signal(signal.SIGTERM, signal_handler)

  # launch run method
  try:
    run(args)
  finally:
    cleanup()

if __name__ == '__main__':
  main()