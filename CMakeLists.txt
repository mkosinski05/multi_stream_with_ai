cmake_minimum_required(VERSION 3.10)

# Set the project name
project(MultiProject)

# Set the C and C++ standards
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED YES)

# Global debug flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -fPIC -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -fPIC -O0")

# List of projects
#set(PROJECTS v4l2_capture gst_capture cv_capture rzv2l_capture vspm_convert)
set(PROJECTS gst_capture2)
# 
# Loop over each project
foreach(PROJECT ${PROJECTS})

    # Collect all .cpp files in the current project directory
    file(GLOB ${PROJECT}_SOURCES "${PROJECT}/*.cpp")
    file(GLOB ${PROJECT}_COMMON "./common/src/*.c")
    
    # Create an executable target for the current project
    add_executable(${PROJECT} ${${PROJECT}_SOURCES} ${${PROJECT}_COMMON})

    # Include directories specific to the current project
    target_include_directories(${PROJECT} PRIVATE ./${PROJECT})
    target_include_directories(${PROJECT} PRIVATE "./common/inc")
    target_include_directories(${PROJECT} PRIVATE $ENV{SDKTARGETSYSROOT}/usr/include)
    target_include_directories(${PROJECT} PRIVATE $ENV{SDKTARGETSYSROOT}/usr/lib64/glib-2.0/include)
    target_include_directories(${PROJECT} PRIVATE $ENV{SDKTARGETSYSROOT}/usr/include/glib-2.0)
    target_include_directories(${PROJECT} PRIVATE $ENV{SDKTARGETSYSROOT}/usr/include/glib-2.0/include)
    target_include_directories(${PROJECT} PRIVATE $ENV{SDKTARGETSYSROOT}/usr/include/opencv4)
    target_include_directories(${PROJECT} PRIVATE $ENV{SDKTARGETSYSROOT}/usr/include/gstreamer-1.0)

    # Link directories
    target_link_directories(${PROJECT} PRIVATE $ENV{SDKTARGETSYSROOT}/usr/lib64)

    # Force Find and link OpenCV
    find_package(OpenCV REQUIRED)
    target_include_directories(${PROJECT} PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${PROJECT} PRIVATE ${OpenCV_LIBS})

    # Find GStreamer
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(GST REQUIRED gstreamer-1.0)
    pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)

    # Include GStreamer headers
    #include_directories(${PROJECT} PRIVATE ${GST_INCLUDE_DIRS})
    #include_directories(${PROJECT} PRIVATE ${GST_APP_INCLUDE_DIRS})
    #message(STATUS ${GST_INCLUDE_DIRS})
    #message(STATUS ${GST_APP_INCLUDE_DIRS})

    # Link GStreamer libraries
    target_link_libraries(${PROJECT} PRIVATE ${GST_LIBRARIES})
    target_link_libraries(${PROJECT} PRIVATE ${GST_APP_LIBRARIES})

    # Add all relevant libraries
    target_link_libraries(${PROJECT} PRIVATE 
        m mmngr mmngrbuf omxr_core pthread vspm
        jpeg webp tiff z tbb gtk-3 png16 gdk-3 cairo
        lzma rt cairo-gobject
        xkbcommon wayland-cursor wayland-egl wayland-client epoxy
        fribidi harfbuzz fontconfig
        glib-2.0 gobject-2.0 gdk_pixbuf-2.0 gmodule-2.0 pangocairo-1.0
        atk-1.0 gio-2.0 pango-1.0 freetype pixman-1 uuid pcre
        mount resolv expat pangoft2-1.0 blkid
        EGL GLESv2
    )

endforeach()
