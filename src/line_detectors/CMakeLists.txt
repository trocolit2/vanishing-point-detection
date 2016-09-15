###################################
## LINE SEGMENT DETETECTOR CMAKE ##
###################################
find_package(OpenCV REQUIRED)

## ADD SUBDIRECTORIES FROM LIBRARY
# include_directories()
# add_subdirectory()

add_library(line_segments_detector  LineSegmentDetector.hpp
                                      aLSD.cpp)

set(SOURCE_LIBS_LOCAL line_segments_detector ${OpenCV_LIBS})

include_directories(aLSD)
add_subdirectory(aLSD)

set(SOURCE_LIBS ${SOURCE_LIBS_LOCAL} ${SOURCE_LIBS} PARENT_SCOPE)
