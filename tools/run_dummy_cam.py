import sys
import signal
import subprocess
import glob
import os
import argparse
import cv2
import time
import math
import numpy as np

INIT_CMDS = [
  'sudo modprobe -r v4l2loopback',
  'sudo modprobe v4l2loopback devices=1 exclusive_caps=1',
]

CLEANUP_TASKS = []

def init(serial_no):
  # reset the module & create loopback device
  for cmd in INIT_CMDS:
    subprocess.run(cmd.split(), capture_output=True, check=True)

  # find the driver
  names = glob.glob('/sys/devices/virtual/video4linux/video*')
  assert len(names) == 1

  [dev_name] = names
  _, dev_path = os.path.split(dev_name)
  dev_path = os.path.join('/dev', dev_path)

  assert os.path.exists(dev_path)

  # create symlink (template: usb-<vendor>_<product>_<serial>-video-index<index>)
  link_name = f'usb-DummyVendor_DummyUSBCamera_{serial_no}-video-index0'
  link_path = os.path.join('/dev/v4l/by-id', link_name)
  os.symlink(dev_path, link_path)

  CLEANUP_TASKS.append(lambda: os.remove(link_path) if os.path.islink(link_path) else None)
  CLEANUP_TASKS.append(lambda: subprocess.run('sudo modprobe -r v4l2loopback'.split(), capture_output=True, check=True))

  return dev_path, link_path

def cleanup():
  [task() for task in CLEANUP_TASKS]

def signal_handler(sig, frame):
  cleanup()
  sys.exit(0)

def run(serial_no):
  dev_path, link_path = init(serial_no)
  print(f'creating dummy camera at {dev_path}')
  print(f'|__ (symlink) {link_path}')

  width = 1280
  height = 720
  fps = 30

  canvas = np.zeros((width, height, 3), dtype=np.uint8)
  proc = subprocess.Popen(
    f'ffmpeg -f rawvideo -pix_fmt bgr24 -s {width}x{height} -i - -f v4l2 -pix_fmt yuyv422 {dev_path}'.split(),
    stdin=subprocess.PIPE,
    stdout=subprocess.DEVNULL,
    stderr=subprocess.STDOUT,
  )

  def kill_ffmpeg():
    proc.stdin.close()
    proc.terminate()

  CLEANUP_TASKS.append(kill_ffmpeg)

  while True:
    start = time.time()
    
    # image generation
    canvas = np.random.randint(0, 256, size=(width, height, 3), dtype=np.uint8)
    proc.stdin.write(canvas.tobytes())
    
    # framerate regulation
    end = time.time()
    time.sleep(max(0, (1 / fps) - (end - start)))

  cleanup()

def main():
  parser = argparse.ArgumentParser(
    prog='dummy_cam',
    description='run dummy v4l2 camera',
  )

  parser.add_argument('-s', '--serial', required=True)
  args = parser.parse_args()

  signal.signal(signal.SIGINT, signal_handler)
  signal.signal(signal.SIGTERM, signal_handler)

  try:
    run(args.serial)
  finally:
    cleanup()

if __name__ == '__main__':
  main()