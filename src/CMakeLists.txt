##################
## SOURCE CMAKE ##
##################
find_package(OpenCV REQUIRED)

## ADD SUBDIRECTORIES FROM LIBRARY
# include_directories()
# add_subdirectory()

add_library(time_count_tools TimeCountTools.hpp)
set_target_properties(time_count_tools PROPERTIES LINKER_LANGUAGE CXX)
set(SOURCE_LIBS_LOCAL time_count_tools ${OpenCV_LIBS})

include_directories(line_detectors)
add_subdirectory(line_detectors)

include_directories(vp_detectors)
add_subdirectory(vp_detectors)


set(SOURCE_LIBS ${SOURCE_LIBS_LOCAL} ${SOURCE_LIBS} PARENT_SCOPE)
