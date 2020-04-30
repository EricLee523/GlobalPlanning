cmake_minimum_required(VERSION 3.10)
project(GlobalPlanning)

set(CMAKE_CXX_STANDARD 11)
# 自动调用moc，uic，rcc处理qt的扩展部分
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# 设置Qt5的cmake模块所在目录，如果不设置将使用系统提供的版本
# QT_DIR和QT_VERSION是指定了qt安装目录和版本的环境变量
# 如果你使用了系统的cmake，那么会优先使用系统提供模块，如果不想发生这种行为你需要自己运行cmake或者使用CLion自带的
set(CMAKE_PREFIX_PATH $ENV{QT_DIR}/$ENV{QT_VERSION}/gcc_64/lib/cmake)

find_package(LibXml2 REQUIRED)
find_package( OpenCV 3 REQUIRED )
# 找到对应的qt模块，名字为qmake中QT += <name>中的name首字母大写后加上Qt5前缀
# 例如core为QtCore，你也可以去${CMAKE_PREFIX_PATH}的目录中找到正确的模块名
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)

# 通常这样设置就可以，如果你的项目包含qrc文件，那么需要将它们单独添加进来
aux_source_directory(. DIRS_SRCS)



include_directories(
        ${LIBXML2_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${Qt5Widgets_INCLUDE_DIRS})


add_library(GPS
        get_gps_data.cpp   get_gps_data.h
        )

target_link_libraries(GPS
        ${catkin_LIBRARIES}
        )

add_executable(GlobalPlanning ${DIRS_SRCS} )

target_link_libraries(GlobalPlanning GPS ${LIBXML2_LIBRARIES} ${OpenCV_LIBS} Qt5::Widgets)