
AUX_SOURCE_DIRECTORY (./ LIB_SRC)

# link_directories(${CMAKE_CURRENT_SOURCE_DIR})
# include_directories(${CMAKE_CURRENT_SOURCE_DIR})
ADD_LIBRARY (iTekCANFD SHARED ${LIB_SRC})

ADD_DEFINITIONS ("-fPIC")

# set (STATIC_LIBS "-Wl,--whole-archive" usb-1.0 "-Wl,--no-whole-archive")

# TARGET_LINK_LIBRARIES(iTekCANFD PUBLIC pthread PRIVATE  ${STATIC_LIBS})
TARGET_LINK_LIBRARIES(iTekCANFD pthread usb-1.0)