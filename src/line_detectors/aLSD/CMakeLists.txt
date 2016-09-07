#######################
## aLSD FOLDER CMAKE ##
#######################
find_package(OpenCV REQUIRED)

add_library(aLSD  lsd.c)

set(SOURCE_LIBS_LOCAL aLSD ${OpenCV_LIBS})
set(SOURCE_LIBS ${SOURCE_LIBS_LOCAL} ${SOURCE_LIBS} PARENT_SCOPE)
