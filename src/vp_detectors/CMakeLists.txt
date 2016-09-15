########################
## VP_DETECTORS CMAKE ##
########################
find_package(OpenCV REQUIRED)

## ADD SUBDIRECTORIES FROM LIBRARY
# include_directories()
# add_subdirectory()

add_library(vp_detectors  VanishingPointDetection.hpp
                          VanishingPointDetectionEvaluation.cpp
                          VanishingPointDetectionTools.cpp)

set_target_properties(vp_detectors PROPERTIES LINKER_LANGUAGE CXX)
set(SOURCE_LIBS_LOCAL vp_detectors ${OpenCV_LIBS})

include_directories(RCSCfMIoMW)
add_subdirectory(RCSCfMIoMW)

set(SOURCE_LIBS ${SOURCE_LIBS_LOCAL} ${SOURCE_LIBS} PARENT_SCOPE)
