##################
## SOURCE CMAKE ##
##################
find_package(OpenCV REQUIRED)

## ADD SUBDIRECTORIES FROM LIBRARY
# include_directories()
# add_subdirectory()

add_library(vanishing_point_detection VanishingPointDetection.hpp
                                      VanishingPointDetectionTools.cpp)

set_target_properties(vanishing_point_detection PROPERTIES LINKER_LANGUAGE CXX)

set(SOURCE_LIBS_LOCAL vanishing_point_detection ${OpenCV_LIBS})
set(SOURCE_LIBS ${SOURCE_LIBS_LOCAL} ${SOURCE_LIBS} PARENT_SCOPE)
