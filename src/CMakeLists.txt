FILE(GLOB_RECURSE COMMON_SOURCE RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.cpp" )
rosbuild_add_executable(%SUBMODEL_NAME% ${COMMON_SOURCE})

