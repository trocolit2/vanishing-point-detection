###############################################################################
## Robust Camera Self-Calibration from Monocular Images of Manhattan Worlds  ##
## 2012                                                                      ##
###############################################################################
find_package(OpenCV REQUIRED)

## ADD SUBDIRECTORIES FROM LIBRARY
# include_directories()
# add_subdirectory()


add_library(RCSCfMIoMW  RCSCfMIoMW.cpp
                        Tools.cpp)

set(SOURCE_LIBS_LOCAL RCSCfMIoMW ${OpenCV_LIBS})
set(SOURCE_LIBS ${SOURCE_LIBS_LOCAL} ${SOURCE_LIBS} PARENT_SCOPE)
