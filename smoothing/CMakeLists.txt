set ( SMOOTHING_EXE_NAME "smoothing" )
set ( MORPHING_TEST "testMorphing")

set ( SMOOTHING_LINK_DIR )
set ( SMOOTHING_INC_DIR ./include)

link_directories(
    ${SMOOTHING_LINK_DIR}
	${C3D_LIBRARY_DIR}
)

include_directories(
    ${SMOOTHING_INC_DIR}
	${C3D_INCLUDE_DIR}
)

include ( SourceFiles.cmake )
add_library( SmoothingLib STATIC ${SMOOTHING_SRC} ${SMOOTHING_INC})

add_executable( ${SMOOTHING_EXE_NAME} ./src/main.cpp ${SMOOTHING_INC} )
add_executable( ${MORPHING_TEST} ./src/test_morphing.cpp ${SMOOTHING_INC} )

target_link_libraries( ${SMOOTHING_EXE_NAME} ${C3D_LIBRARIES} SmoothingLib)
#target_link_libraries( ${MORPHING_TEST} ${C3D_LIBRARIES} SmoothingLib)
target_include_directories( ${MORPHING_TEST} PUBLIC C:/myprogram/geomView/install/include)
target_link_libraries( ${MORPHING_TEST} ${C3D_LIBRARIES} opengl32.lib SmoothingLib C:/myprogram/geomView/install/lib/libgeom_view.lib)

set_target_properties( ${SMOOTHING_EXE_NAME}
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/../bin/Release"
    LIBRARY_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/../lib/Release"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_BINARY_DIR}/../bin/Release"
    ARCHIVE_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/../bin/Debug"
    LIBRARY_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/../lib/Debug"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_BINARY_DIR}/../bin/Debug"
)