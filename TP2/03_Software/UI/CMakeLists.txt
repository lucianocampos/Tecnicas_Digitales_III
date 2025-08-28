cmake_minimum_required(VERSION 3.16)
project(TP2_STM32 LANGUAGES CXX)

# 1) Desactivar response files en MinGW para evitar el error de cc1plus.exe
if (MINGW)
  set(CMAKE_CXX_USE_RESPONSE_FILE OFF)
  set(CMAKE_CXX_USE_RESPONSE_FILE_FOR_INCLUDES OFF)
  set(CMAKE_C_USE_RESPONSE_FILE OFF)
  set(CMAKE_C_USE_RESPONSE_FILE_FOR_INCLUDES OFF)
endif()

set(CMAKE_CXX_STANDARD 17)

# Qt autogen
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets SerialPort)

qt_standard_project_setup()

# 2) No incluimos el header en sources, Qt lo gestionar por MOC si es necesario
qt_add_executable(${PROJECT_NAME}
    main.cpp
    mainwindow.cpp
    mainwindow.ui
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    WIN32_EXECUTABLE TRUE
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE Qt6::Widgets Qt6::SerialPort
)

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
)

# Opcional: clang-tidy
set(CMAKE_CXX_CLANG_TIDY
    "clang-tidy;
     -checks=-clazy-connect-by-name")
