import sys
import signal
import subprocess
import glob
import os
import argparse
import time
import re
import math
import ctypes
import faulthandler
from pymodule.raytrace import init_raytrace, Camera

# debugging
faulthandler.enable()

# initialize the raytracing module
raytrace = init_raytrace()

DEFAULT_VENDOR = 'DummyVendor'
DEFAULT_PRODUCT_NAME = 'DummyV4L2Camera'
DEFAULT_SERIAL_NO = 'DUMMY_SERIAL_NO'
DEFAULT_SIZE = (1280, 720)
DEFAULT_FPS = '30'

SIZE_RE = r'^(?P<width>\d+)x(?P<height>\d+)\Z'

CLEANUP_TASKS = []
INIT_CMDS = [
  'sudo modprobe -r v4l2loopback',
  'sudo modprobe v4l2loopback devices=1 exclusive_caps=1',
]

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
  proc = subprocess.Popen(
    f'ffmpeg -f rawvideo -pix_fmt rgb24 -s {width}x{height} -i - -f v4l2 -pix_fmt yuyv422 {dev_path}'.split(),
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

  # TODO: figure out why this segfaults
  # CLEANUP_TASKS.append(lambda: raytrace.free_image_buffer(buffer_ptr))
  
  # TODO: load camera YAML
  camera = Camera(
    (ctypes.c_int * 2)(width, height),
    (ctypes.c_float * 4)(width / 2, height / 2, 1032.0, 1032.0),
    (ctypes.c_float * 8)(-0.5, 0.12, 0.0001, -0.0005, -0.0227, 0, 0, 0),
  )

  raytrace.init_workers()
  
  while True:
    start = time.time()
    
    # image generation
    raytrace.render_frame(buffer_ptr, ctypes.byref(camera), 2*math.pi*(start - int(start)))
    proc.stdin.write(ctypes.string_at(buffer_ptr, buffer_size))
    
    # framerate regulation
    end = time.time()
    time.sleep(max(0, (1 / fps) - (end - start)))

  # cleanup()

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