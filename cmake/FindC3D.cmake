if ( C3D_FOUND )
    return()
endif()

message (WARNING "=== C3D_DIR === [${C3D_DIR}]" )
message (WARNING "=== NAMES === [c3d-${UNISYS_C3D_VERSION}-${UNISYS_COMPILER_PLATFORM}]"  )
message (WARNING "=== UNISYS_SDK2_DIR === [${UNISYS_SDK2_DIR}] [${UNISYS_SDK_DIR}]"  )


# Поиск каталога библиотеки
find_file ( C3D_DIR 
    NAMES c3d-${UNISYS_C3D_VERSION}-${UNISYS_COMPILER_PLATFORM}
    PATHS "${UNISYS_SDK2_DIR}" "${UNISYS_SDK_DIR}" 
    PATH_SUFFIXES C3D c3d
    NO_DEFAULT_PATH 
)

message ( "C3D_DIR = [${C3D_DIR}]" )

find_package ( C3D ${UNISYS_C3D_VERSION} 
    PATHS "${C3D_DIR}" 
    NO_DEFAULT_PATH 
)

# Поиск и копирование файлов библиотеки в каталог проекта
EXPORT_C3D_LIBRARIES ( Debug   "${CMAKE_BINARY_DIR}/../bin/Debug"   )
EXPORT_C3D_LIBRARIES ( Release "${CMAKE_BINARY_DIR}/../bin/Release" )
# Установка зависимостей в фазе инсталяции
install ( FILES ${C3D_LIBRARY_Debug}   DESTINATION bin/Debug   CONFIGURATIONS Debug   )
install ( FILES ${C3D_LIBRARY_Release} DESTINATION bin/Release CONFIGURATIONS Release )

