cmake_minimum_required(VERSION 3.15)
project(demo01)

set(CMAKE_CXX_STANDARD 14)

#add_library(glog STATIC IMPORTED)
#set_property(TARGET glog PROPERTY IMPORTED_LOCATION ~/CLionProjects/demo01/lib/libglog.a)


include_directories(
        /home/ethan/absl/abseil-cpp/install/include
        /home/ethan/cartographer_ws/install_isolated/include
        /usr/include/lua5.2
        /home/ethan/cartographer_without_ros/cartographer_superbuild/ros_stub/include
)

link_directories(
        /usr/lib
        /usr/local/lib
        /usr/local/protobuf/lib
        /usr/lib/x86_64-linux-gnu
        /usr/lib/x86_64-linux-gnu/ruby/2.3.0
        /home/ethan/cartographer_ws/install_isolated/lib
)

add_executable(demo01 main.cpp)

target_link_libraries(demo01 libglog.a libgflags.a libunwind.a liblzma.a
        /home/ethan/cartographer_ws/install_isolated/lib/libcartographer_ros.a
        /home/ethan/cartographer_ws/install_isolated/lib/libcartographer.a
        /usr/local/lib/libprotobuf.a
        /home/ethan/cartographer_ws/install_isolated/lib/libceres.a
        /home/ethan/lib64/libabsl_strings.a
        /home/ethan/lib64/libabsl_str_format_internal.a
        /home/ethan/lib64/libabseil.a
        /usr/lib/x86_64-linux-gnu/libboost_iostreams.a
        /usr/lib/x86_64-linux-gnu/liblua5.2.a
        -lz
        -ldl
        /usr/lib/gcc/x86_64-linux-gnu/5/libgomp.a
        -lcholmod
        /usr/lib/x86_64-linux-gnu/libcxsparse.a
        -llapack
        libblas.a
        -lpthread
        /usr/lib/x86_64-linux-gnu/libcairo.so.2
        /usr/lib/x86_64-linux-gnu/libpixman-1.a
        /usr/lib/x86_64-linux-gnu/libboost_system.a
        /usr/lib/x86_64-linux-gnu/libfontconfig.a
        /usr/lib/x86_64-linux-gnu/libfreetype.a
        /usr/lib/x86_64-linux-gnu/libpng.a
        /usr/lib/x86_64-linux-gnu/libexpat.a
        /opt/ros/kinetic/lib/librostime.so
        ${CMAKE_THREAD_LIBS_INIT})
