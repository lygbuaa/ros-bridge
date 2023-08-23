# itekon-canfd

#### 介绍
canfd linux 软件驱动

#### 运行依赖
libusb-1.0

#### 编译步骤

mkdir build

cd build

cmake ../;make

#### 动态库编译步骤

gcc canfd_test.c -I ./lib.`uname -m`/ -L ./lib.`uname -m`/ -lusb-1.0 -lpthread -liTekCANFD

