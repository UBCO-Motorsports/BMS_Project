cmake_minimum_required(VERSION 3.5)
project(BMS_GUI LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_PREFIX_PATH "C:/msys64/mingw64")

# ADD SerialPort to the required components
find_package(Qt5 COMPONENTS Widgets SerialPort REQUIRED)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

include_directories(include)

add_executable(BMS_GUI
    src/main_gui.cpp
    src/MainWindow.cpp
    include/MainWindow.h   
    src/MainWindow.ui      
    # NOTICE: We removed the mock folder and all your FSM source files! 
    # The GUI is now just a dumb terminal listening to the hardware.
)

# Link the SerialPort library
target_link_libraries(BMS_GUI PRIVATE Qt5::Widgets Qt5::SerialPort)