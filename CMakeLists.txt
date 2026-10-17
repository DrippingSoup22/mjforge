cmake_minimum_required(VERSION 3.21)
project(MJForge VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(MJFORGE_BUILD_TESTS "Build unit tests" ON)
option(MJFORGE_USE_SYSTEM_GLFW "Use an installed GLFW instead of downloading it" OFF)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
if(APPLE)
  set(CMAKE_BUILD_RPATH "@loader_path")
  set(CMAKE_INSTALL_RPATH "@loader_path")
elseif(UNIX)
  set(CMAKE_BUILD_RPATH "$ORIGIN")
  set(CMAKE_INSTALL_RPATH "$ORIGIN")
endif()

include(FetchContent)
include(cmake/Embed.cmake)
include(cmake/MuJoCo.cmake)
find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)

# ---------------------------------------------------------------------------------------------
# Third-party dependencies
# ---------------------------------------------------------------------------------------------
if(MJFORGE_USE_SYSTEM_GLFW)
  find_package(glfw3 3.3 REQUIRED)
else()
  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
  # X11 works everywhere (also under Wayland through XWayland); enable Wayland explicitly if wanted.
  set(GLFW_BUILD_WAYLAND OFF CACHE BOOL "Build GLFW with Wayland support")
  FetchContent_Declare(glfw
    URL https://github.com/glfw/glfw/releases/download/3.4/glfw-3.4.zip
    URL_HASH SHA256=b5ec004b2712fd08e8861dc271428f048775200a2df719ccf575143ba749a3e9
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  FetchContent_MakeAvailable(glfw)
endif()

FetchContent_Declare(imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG v1.93.0-docking
  GIT_SHALLOW TRUE)
FetchContent_Declare(implot
  GIT_REPOSITORY https://github.com/epezent/implot.git
  GIT_TAG v1.0
  GIT_SHALLOW TRUE)
FetchContent_Declare(imguizmo
  GIT_REPOSITORY https://github.com/CedricGuillemet/ImGuizmo.git
  GIT_TAG 105e4fde45e4e74aae8973d98750820ef7aca7e2
  SOURCE_SUBDIR do-not-add)
FetchContent_Declare(nlohmann_json
  URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
  URL_HASH SHA256=42f6e95cad6ec532fd372391373363b62a14af6d771056dbfc86160e6dfff7aa
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(imgui implot imguizmo nlohmann_json)

add_library(mjf_imgui STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/imgui_demo.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
  ${imgui_SOURCE_DIR}/misc/cpp/imgui_stdlib.cpp
  ${implot_SOURCE_DIR}/implot.cpp
  ${implot_SOURCE_DIR}/implot_items.cpp
  ${imguizmo_SOURCE_DIR}/src/ImGuizmo.cpp)
target_include_directories(mjf_imgui PUBLIC
  ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends ${imgui_SOURCE_DIR}/misc/cpp
  ${implot_SOURCE_DIR} ${imguizmo_SOURCE_DIR}/src)
target_compile_definitions(mjf_imgui PUBLIC IMGUI_DEFINE_MATH_OPERATORS)
target_link_libraries(mjf_imgui PUBLIC glfw OpenGL::GL)

add_library(mjf_glad STATIC third_party/glad/src/gl.c)
target_include_directories(mjf_glad PUBLIC third_party/glad/include)

add_library(mjf_stb INTERFACE)
target_include_directories(mjf_stb INTERFACE third_party/stb)

# Embedded fonts.
set(MJF_EMBED_CPP "${CMAKE_BINARY_DIR}/generated/embedded_assets.cpp")
mjforge_embed_files("${MJF_EMBED_CPP}"
  mjf_font_ui "${CMAKE_SOURCE_DIR}/assets/fonts/Inter-Medium.subset.otf"
  mjf_font_mono "${CMAKE_SOURCE_DIR}/assets/fonts/JetBrainsMono-Regular.subset.ttf")

# Compiler warnings for our own code.
function(mjforge_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W3 /permissive- /utf-8)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers)
  endif()
endfunction()

# ---------------------------------------------------------------------------------------------
# MJForge core: document model, MJCF export/import, analysis, MuJoCo bridge. No GL / UI.
# ---------------------------------------------------------------------------------------------
file(GLOB MJF_CORE_SOURCES CONFIGURE_DEPENDS
  src/core/*.cpp src/analysis/*.cpp src/sim/*.cpp)
add_library(mjforge_core STATIC ${MJF_CORE_SOURCES})
target_include_directories(mjforge_core PUBLIC src)
target_link_libraries(mjforge_core PUBLIC mujoco::mujoco nlohmann_json::nlohmann_json Threads::Threads)
target_compile_definitions(mjforge_core PUBLIC
  MJFORGE_VERSION="${PROJECT_VERSION}"
  MJFORGE_MUJOCO_VERSION="${MJFORGE_MUJOCO_VERSION_FOUND}")
mjforge_warnings(mjforge_core)

# ---------------------------------------------------------------------------------------------
# MJForge application
# ---------------------------------------------------------------------------------------------
file(GLOB MJF_APP_SOURCES CONFIGURE_DEPENDS
  src/app/*.cpp src/render/*.cpp src/ui/*.cpp)
add_executable(mjforge ${MJF_APP_SOURCES} ${MJF_EMBED_CPP})
target_link_libraries(mjforge PRIVATE mjforge_core mjf_imgui mjf_glad mjf_stb)
mjforge_warnings(mjforge)
if(WIN32)
  set_target_properties(mjforge PROPERTIES WIN32_EXECUTABLE $<CONFIG:Release>)
  if(MSVC)
    target_link_options(mjforge PRIVATE $<$<CONFIG:Release>:/ENTRY:mainCRTStartup>)
  endif()
endif()

# Ship the MuJoCo runtime next to the executable.
add_custom_command(TARGET mjforge POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${MJFORGE_MUJOCO_RUNTIME_FILE}" "$<TARGET_FILE_DIR:mjforge>"
  VERBATIM)

install(TARGETS mjforge RUNTIME DESTINATION bin)
install(FILES "${MJFORGE_MUJOCO_RUNTIME_FILE}" DESTINATION bin)

# ---------------------------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------------------------
if(MJFORGE_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
