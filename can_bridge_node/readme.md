## Prerequisites
1. `sudo apt install can-utils`  #cansend, candump

## test
1. `bash tools/build_can_bridge.sh`
2. `bash tools/test_can_bridge.sh`

## run chuang-xin-ke-ji usbcan demo controlcan
1. start docker with usb support and privileged mode:  
    `  docker run --privileged --gpus='all,"capabilities=compute,utility,graphics,display"' -it -v `pwd`:`pwd` -w `pwd` --ipc=host --device /dev/dri -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY  --env DISPLAY:$DISPLAY -e GDK_SCALE -e GDK_DPI_SCALE --network host -v /dev/bus/usb:/dev/bus/usb osrf/ros:foxy-ue4 /bin/bash  `
2. `cd usbcan/controlcan`
3. `make clean && make`
4. `sudo ./hello_cpp`

## run itekon canfd demo
1. start docker with usb support and privileged mode
2. `apt install libusb-1.0-0-dev`
3. `cd usbcan/itekon_canfd_linux`
4. `mkdir build && cd build`
5. `cmake .. && make`
6. `sudo ./canfd-test`


## notice
1. set USBCAN_MODEL, in CMakeLists.txt
2. set baudrate manually, in open_usbcan().
3. if using canfd with itekon x200, call open_usbcan(x, 1) and choose CanfdFrame_t.
