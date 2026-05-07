cmake_minimum_required(VERSION 3.16)
project(AngryBird)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Generate compile_commands.json for clangd/IDE
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find GLFW (installed via Homebrew: brew install glfw)
find_package(glfw3 REQUIRED)

# Find OpenGL framework on macOS
find_package(OpenGL REQUIRED)

# Source files
set(SOURCES
    src/main.cpp
    src/Renderer.cpp
    src/Physics.cpp
    src/Game.cpp
)

# Create executable
add_executable(AngryBird ${SOURCES})

# Include directories
target_include_directories(AngryBird PRIVATE
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/third_party/stb
)

target_compile_definitions(AngryBird PRIVATE
    ANGRY_BIRD_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

# Link libraries
target_link_libraries(AngryBird PRIVATE
    glfw
    OpenGL::GL
)

# macOS specific frameworks
if(APPLE)
    target_link_libraries(AngryBird PRIVATE
        "-framework Cocoa"
        "-framework IOKit"
    )
endif()