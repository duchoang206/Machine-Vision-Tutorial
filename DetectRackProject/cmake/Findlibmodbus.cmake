# - Try to find libmodbus
# Once done this will define
#
#  libmodbus_FOUND - system has libmodbus
#  libmodbus_INCLUDE_DIRS - the libmodbus include directory
#  libmodbus_LIBRARIES - Link these to use libmodbus

find_path(libmodbus_INCLUDE_DIR
    NAMES modbus/modbus.h modbus.h
    PATH_SUFFIXES modbus
)

find_library(libmodbus_LIBRARY
    NAMES modbus libmodbus
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(libmodbus
    DEFAULT_MSG
    libmodbus_LIBRARY
    libmodbus_INCLUDE_DIR
)

if(libmodbus_FOUND)
    set(libmodbus_LIBRARIES ${libmodbus_LIBRARY})
    set(libmodbus_INCLUDE_DIRS ${libmodbus_INCLUDE_DIR})
    
    if(NOT TARGET libmodbus::libmodbus)
        add_library(libmodbus::libmodbus UNKNOWN IMPORTED)
        set_target_properties(libmodbus::libmodbus PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${libmodbus_INCLUDE_DIR}"
            IMPORTED_LOCATION "${libmodbus_LIBRARY}"
        )
    endif()
endif()

mark_as_advanced(libmodbus_INCLUDE_DIR libmodbus_LIBRARY)
