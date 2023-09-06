## prepare env
1. run docker: 
` docker run --privileged --gpus all -it -v `pwd`:`pwd` -w `pwd` --ipc=host \
--device  /dev/dri	\
--device  /dev/xdma0_bypass	\
--device  /dev/xdma0_bypass_c2h_0 \
--device  /dev/xdma0_bypass_h2c_0 \
--device  /dev/xdma0_c2h_0  	\
--device  /dev/xdma0_control  	\
--device  /dev/xdma0_h2c_0  	\
--device  /dev/xdma0_user  	\
--device  /dev/xdma0_xvc		\
--device  /dev/xdma0_events_0   	\
--device  /dev/xdma0_events_1  	\
--device  /dev/xdma0_events_2  	\
--device  /dev/xdma0_events_3  	\
--device  /dev/xdma0_events_4  	\
--device  /dev/xdma0_events_5  	\
--device  /dev/xdma0_events_6  	\
--device  /dev/xdma0_events_7  	\
--device  /dev/xdma0_events_8  	\
--device  /dev/xdma0_events_9 	\
--device  /dev/xdma0_events_10    \
--device  /dev/xdma0_events_11  	\
--device  /dev/xdma0_events_12  	\
--device  /dev/xdma0_events_13  	\
--device  /dev/xdma0_events_14   	\
--device  /dev/xdma0_events_15	\
--device  /dev/xdma1_bypass	\
--device  /dev/xdma1_bypass_c2h_0 \
--device  /dev/xdma1_bypass_h2c_0 \
--device  /dev/xdma1_c2h_0  	\
--device  /dev/xdma1_control  	\
--device  /dev/xdma1_h2c_0  	\
--device  /dev/xdma1_user  	\
--device  /dev/xdma1_xvc		\
--device  /dev/xdma1_events_0   	\
--device  /dev/xdma1_events_1  	\
--device  /dev/xdma1_events_2  	\
--device  /dev/xdma1_events_3  	\
--device  /dev/xdma1_events_4  	\
--device  /dev/xdma1_events_5  	\
--device  /dev/xdma1_events_6  	\
--device  /dev/xdma1_events_7  	\
--device  /dev/xdma1_events_8  	\
--device  /dev/xdma1_events_9 	\
--device  /dev/xdma1_events_10    \
--device  /dev/xdma1_events_11  	\
--device  /dev/xdma1_events_12  	\
--device  /dev/xdma1_events_13  	\
--device  /dev/xdma1_events_14   	\
--device  /dev/xdma1_events_15	\
-v /etc/localtime:/etc/localtime:ro \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-e DISPLAY=unix$DISPLAY -e GDK_SCALE -e GDK_DPI_SCALE \
--network host 	\
osrf/ros:foxy-carla /bin/bash `

## HDMI blanking interval
1. signal_w / signal_h in channel_create() needs to consider the HDMI blanking interval (https://en.wikipedia.org/wiki/Vertical_blanking_interval)
2. here is a calculator for h_total / v_total:  https://tomverbeure.github.io/video_timings_calculator
3. confirmed pairs:
    3840*2160 -> 4400*2250
    1920*1080 -> 2200*1125
    1280*720 -> 1650*750
    640*360 -> 800*375