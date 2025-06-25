cmake_minimum_required(VERSION 3.10)

project(quadcopter_simulation CXX)

# Set the C++ standard to C++11 or later (adjust if needed)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Define source files
set(SOURCE_FILES
    src/accelerometer.cpp
    src/barometer.cpp
    src/complementaryFilter.cpp
    src/gyroscope.cpp
    src/magnetometer.cpp
    src/main.cpp
    src/esc_motor.cpp
    src/stabilizer.cpp
    src/PID.cpp
    src/quadcopter.cpp
    src/irr_quadcopter_3D.cpp
    src/receiver.cpp
    src/irr_renderer.cpp
    src/sensorFusion.cpp
    src/timer.cpp
    src/utils_diffequation.cpp
)

# Define header files (optional for compilation, but good for IDEs)
set(HEADER_FILES
    src/barometer.h
    src/irr_event_receiver.h
    src/irr_keytimer.h
    src/main_config.h
    src/complementaryFilter.h
    src/config.h
    src/accelerometer.h
    src/irr_coordinate_axes.h
    src/gyroscope.h
    src/magnetometer.h
    src/esc_motor.h
    src/stabilizer.h
    src/PID.h
    src/quadcopter.h
    src/irr_quadcopter_3D.h
    src/receiver.h
    src/irr_renderer.h
    src/sensorFusion.h
    src/timer.h
    src/mathHelp.h
    src/utils_diffequation.h
    src/vectorTypes.h
)

# Add the executable
add_executable(quadcopter_simulation ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(quadcopter_simulation
    Irrlicht
    pthread
)

# Set output directories for executables
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
