# micropython.cmake
# Ublox NMEA module CMake configuration
target_sources(usermod INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/ublox_nmea.c
)

target_include_directories(usermod INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)