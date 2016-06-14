##################
## SOURCE CMAKE ##
##################
find_package(OpenCV REQUIRED)

## ADD SUBDIRECTORIES FROM LIBRARY
# include_directories()
# add_subdirectory()

add_library(vpd_library VanishingPointDetection.hpp)
set_target_properties(vpd_library PROPERTIES LINKER_LANGUAGE CXX)

set(SOURCE_LIBS_LOCAL ps4eyeLibrary ${OpenCV_LIBS} )
set(SOURCE_LIBS ${SOURCE_LIBS_LOCAL} ${SOURCE_LIBS} PARENT_SCOPE)
