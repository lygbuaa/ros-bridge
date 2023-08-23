cmake_minimum_required(VERSION 2.8)

project(canfd)

ADD_SUBDIRECTORY (lib)
INCLUDE_DIRECTORIES (lib)

add_executable(canfd-test canfd_test.c)

add_definitions("-fpermissive")

target_link_libraries (canfd-test iTekCANFD)
