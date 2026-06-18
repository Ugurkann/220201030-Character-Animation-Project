cmake_minimum_required(VERSION 3.20)

project(student_character_plugin
    VERSION 1.0.0
    LANGUAGES CXX
)

add_library(CharacterProject SHARED
    joint_controller.cpp
    plugin.cpp
)

target_compile_features(CharacterProject PRIVATE cxx_std_17)

target_include_directories(CharacterProject
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

set_target_properties(CharacterProject PROPERTIES
    OUTPUT_NAME "character_plugin_ugurkan"
    PREFIX ""
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/out"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/out"
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/out"
)

foreach(CONFIGURATION Debug Release RelWithDebInfo MinSizeRel)
    string(TOUPPER "${CONFIGURATION}" CONFIGURATION_UPPER)
    set_target_properties(CharacterProject PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY_${CONFIGURATION_UPPER} "${CMAKE_CURRENT_SOURCE_DIR}/out"
        LIBRARY_OUTPUT_DIRECTORY_${CONFIGURATION_UPPER} "${CMAKE_CURRENT_SOURCE_DIR}/out"
        ARCHIVE_OUTPUT_DIRECTORY_${CONFIGURATION_UPPER} "${CMAKE_CURRENT_SOURCE_DIR}/out"
    )
endforeach()

if(MSVC)
    target_compile_options(CharacterProject PRIVATE /W4 /permissive- /EHsc)
    target_compile_definitions(CharacterProject PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX WIN32_LEAN_AND_MEAN)
else()
    target_compile_options(CharacterProject PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(CharacterProjectTest
    test_controller.cpp
    plugin.cpp
    joint_controller.cpp
)

target_compile_features(CharacterProjectTest PRIVATE cxx_std_17)

target_include_directories(CharacterProjectTest
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

set_target_properties(CharacterProjectTest PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/out"
)

foreach(CONFIGURATION Debug Release RelWithDebInfo MinSizeRel)
    string(TOUPPER "${CONFIGURATION}" CONFIGURATION_UPPER)
    set_target_properties(CharacterProjectTest PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY_${CONFIGURATION_UPPER} "${CMAKE_CURRENT_SOURCE_DIR}/out"
    )
endforeach()

if(MSVC)
    target_compile_options(CharacterProjectTest PRIVATE /W4 /permissive- /EHsc)
    target_compile_definitions(CharacterProjectTest PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX WIN32_LEAN_AND_MEAN)
else()
    target_compile_options(CharacterProjectTest PRIVATE -Wall -Wextra -Wpedantic)
endif()
