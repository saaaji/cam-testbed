# cam-testbed
Testing for camera backend for CU-Robotics depth camera.
Useful for when access to hardware is limited and 
can test on a cheap USB camera.

### Software testbed for debugging camera backend features:
- `video4linux` capture backend (mmap'd)
  - buffered to keep up with camera framerate, but only latest buffer is  used (to enable real-time applications)
- probing `/dev` for desired camera based on serial number
- recovery of device nodes via `libudev` polling in the event of lost USB connection

### Building:
- create a `/build` directory, `cd` into it & run `cmake ..`
- use `make` to build from now on
