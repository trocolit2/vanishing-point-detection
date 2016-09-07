########################
## VP_DETECTORS CMAKE ##
########################
find_package(OpenCV REQUIRED)

## ADD SUBDIRECTORIES FROM LIBRARY
# include_directories()
# add_subdirectory()


add_library(vp_detectors  VanishingPointDetection.hpp
                          RCSCfMIoMW.cpp)

set_target_properties(vp_detectors PROPERTIES LINKER_LANGUAGE CXX)


set(SOURCE_LIBS_LOCAL vp_detectors ${OpenCV_LIBS})
set(SOURCE_LIBS ${SOURCE_LIBS_LOCAL} ${SOURCE_LIBS} PARENT_SCOPE)
