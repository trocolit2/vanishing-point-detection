##################
## SOURCE CMAKE ##
##################
find_package(OpenCV REQUIRED)

## ADD SUBDIRECTORIES FROM LIBRARY
# include_directories()
# add_subdirectory()

add_library(vanishing_point_detection   VanishingPointDetectionTools.cpp
                                        VanishingPointDetectionEvaluation.cpp)


set(SOURCE_LIBS_LOCAL vanishing_point_detection ${OpenCV_LIBS})

include_directories(line_detectors)
add_subdirectory(line_detectors)

include_directories(vp_detectors)
add_subdirectory(vp_detectors)

set(SOURCE_LIBS ${SOURCE_LIBS_LOCAL} ${SOURCE_LIBS} PARENT_SCOPE)
