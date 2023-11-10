cmake_minimum_required(VERSION 3.10)

add_library(_TOFDPort SHARED IMPORTED GLOBAL)

set_target_properties(
    _TOFDPort PROPERTIES 
    IMPORTED_LOCATION "${CMAKE_CURRENT_LIST_DIR}/lib/TOFDPort.dll"
    IMPORTED_IMPLIB "${CMAKE_CURRENT_LIST_DIR}/lib/TOFDPort.lib"
)

add_library(TOFDPort TOFDPort.cpp)

target_include_directories(TOFDPort PUBLIC "include" ${HDBridge_INCLUDE})

target_link_libraries(TOFDPort PUBLIC _TOFDPort)

if (CMAKE_USE_SQLITE_ORM)
    message(STATUS TOFDPort: use sqlite orm)
    target_link_libraries(TOFDPort PUBLIC ORM::SQLite3)
endif()


add_library(HDBridge::TOFDPort ALIAS TOFDPort)

if (Morose_FOUND)
    morose_copy(
        TARGET 
        TOFDPort 
        FILES 
        "${CMAKE_CURRENT_LIST_DIR}/lib/TOFDPort.dll"
        "${CMAKE_CURRENT_LIST_DIR}/lib/ftd2xx.dll"
    )
endif()
