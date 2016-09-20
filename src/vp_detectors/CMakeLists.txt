########################
## VP_DETECTORS CMAKE ##
########################
find_package(OpenCV REQUIRED)

## ADD SUBDIRECTORIES FROM LIBRARY
# include_directories()
# add_subdirectory()

add_library(RCSCfMIoMW  RCSCfMIoMW/RCSCfMIoMW.cpp
                        RCSCfMIoMW/Tools.cpp)

add_library(vp_detectors  VanishingPointDetection.hpp
                          VanishingPointDetectionEvaluation.cpp
                          VanishingPointDetectionTools.cpp)

set_target_properties(vp_detectors PROPERTIES LINKER_LANGUAGE CXX)


set(SOURCE_LIBS_LOCAL RCSCfMIoMW vp_detectors ${OpenCV_LIBS})

set(SOURCE_LIBS ${SOURCE_LIBS_LOCAL} ${SOURCE_LIBS} PARENT_SCOPE)
