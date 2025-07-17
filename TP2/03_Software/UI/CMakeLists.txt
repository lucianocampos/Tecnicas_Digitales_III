cmake_minimum_required(VERSION 3.16)

set(CMAKE_RESPONSE_FILE_LINK_FLAG "")  # Desactiva @response files en linking
set(CMAKE_CXX_USE_RESPONSE_FILE_FOR_OBJECTS OFF)
set(CMAKE_CXX_USE_RESPONSE_FILE_FOR_INCLUDES OFF)

project(TP2_STM32 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets SerialPort)

qt_standard_project_setup()

qt_add_executable(${PROJECT_NAME}
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
)

set_target_properties(${PROJECT_NAME} PROPERTIES WIN32_EXECUTABLE TRUE)

target_link_libraries(${PROJECT_NAME}
    PRIVATE Qt6::Widgets Qt6::SerialPort
)

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
)

set(CMAKE_CXX_CLANG_TIDY
    "clang-tidy;
     -checks=-clazy-connect-by-name")





