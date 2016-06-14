#####################
## TEST MAIN CMAKE ##
#####################

# include tools libraries from test cases
#include_directories("tools")
#add_subdirectory("tools")

find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS unit_test_framework REQUIRED)
include_directories (${Boost_INCLUDE_DIRS})
set(SOURCE_LIBS ${SOURCE_LIBS} "-std=c++0x -fpermissive")

#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()

file(GLOB TEST_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *test.cpp)

# make all the tests bins
foreach(testSrc ${TEST_FILES})
        #Extract the filename without an extension (NAME_WE)
        get_filename_component(testName ${testSrc} NAME_WE)

        #Add compile target
        add_executable(${testName} ${testSrc})

        #link to Boost libraries AND your targets and dependencies
        target_link_libraries(${testName} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${SOURCE_LIBS})

        #I like to move testing binaries into a testBin directory
        set_target_properties(${testName} PROPERTIES
                 RUNTIME_OUTPUT_DIRECTORY ${bin_path}/testbin)

        #Finally add it to test execution -
        #Notice the WORKING_DIRECTORY and COMMAND
        add_test(NAME ${testName}
                WORKING_DIRECTORY ${bin_path}/testbin
                COMMAND ${bin_path}/testbin/${testName})
endforeach(testSrc)
