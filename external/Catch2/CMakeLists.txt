if(NOT EXISTS "${CMAKE_CURRENT_BINARY_DIR}/single_include/catch2/catch.hpp")
    message("catch.hpp not in ${CMAKE_CURRENT_BINARY_DIR}/single_include/catch2, download now......")
    file(DOWNLOAD https://github.com/catchorg/Catch2/releases/download/v2.13.7/catch.hpp
            "${CMAKE_CURRENT_BINARY_DIR}/single_include/catch2/catch.hpp"
            )
endif()

add_library(Catch2 INTERFACE)
add_library(Catch2::Catch2 ALIAS Catch2)
target_include_directories(Catch2
        INTERFACE
        "${CMAKE_CURRENT_BINARY_DIR}/single_include"
        )