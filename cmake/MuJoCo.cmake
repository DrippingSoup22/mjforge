# Locates (or downloads) MuJoCo and defines the imported target `mujoco::mujoco`.
#
# Search order:
#   1. MUJOCO_DIR: either an official release folder (include/ + lib/ or bin/) or the folder of
#      the Python `mujoco` package (contains libmujoco.so.X.Y.Z and include/).
#   2. The Python `mujoco` package found via the Python3 interpreter (MJFORGE_MUJOCO_FROM_PYTHON).
#      This is the recommended setup: the editor then validates with exactly the same MuJoCo
#      version that MuJoCo Warp uses when loading the exported XML.
#   3. Download of the official release MJFORGE_MUJOCO_VERSION (Linux x86_64/aarch64, Windows).
#
# Outputs:
#   mujoco::mujoco                IMPORTED SHARED target
#   MJFORGE_MUJOCO_RUNTIME_FILE   shared library to copy next to the executable
#   MJFORGE_MUJOCO_VERSION_FOUND  version string parsed from mujoco.h

set(MJFORGE_MUJOCO_VERSION "3.16.0" CACHE STRING "MuJoCo release to download if no local copy is found")
set(MUJOCO_DIR "" CACHE PATH "MuJoCo release folder or Python 'mujoco' package folder")
option(MJFORGE_MUJOCO_FROM_PYTHON "Use the MuJoCo library shipped inside the Python 'mujoco' package if available" ON)

function(_mjforge_probe_mujoco dir out_found out_inc out_lib out_implib)
  set(${out_found} FALSE PARENT_SCOPE)
  if(NOT dir OR NOT EXISTS "${dir}/include/mujoco/mujoco.h")
    return()
  endif()
  set(implib "")
  if(WIN32)
    file(GLOB libs "${dir}/bin/mujoco.dll" "${dir}/mujoco.dll")
    file(GLOB implibs "${dir}/lib/mujoco.lib" "${dir}/mujoco.lib")
    if(NOT libs OR NOT implibs)
      return()
    endif()
    list(GET implibs 0 implib)
  elseif(APPLE)
    file(GLOB libs "${dir}/lib/libmujoco*.dylib" "${dir}/libmujoco*.dylib")
  else()
    file(GLOB libs "${dir}/lib/libmujoco.so.*" "${dir}/libmujoco.so.*" "${dir}/lib/libmujoco.so" "${dir}/libmujoco.so")
  endif()
  if(NOT libs)
    return()
  endif()
  list(SORT libs)
  list(GET libs 0 lib)
  set(${out_found} TRUE PARENT_SCOPE)
  set(${out_inc} "${dir}/include" PARENT_SCOPE)
  set(${out_lib} "${lib}" PARENT_SCOPE)
  set(${out_implib} "${implib}" PARENT_SCOPE)
endfunction()

set(_mj_found FALSE)

# 1. Explicit folder.
if(MUJOCO_DIR)
  _mjforge_probe_mujoco("${MUJOCO_DIR}" _mj_found _mj_inc _mj_lib _mj_implib)
  if(NOT _mj_found)
    message(FATAL_ERROR "MUJOCO_DIR='${MUJOCO_DIR}' does not contain include/mujoco/mujoco.h and a MuJoCo library.")
  endif()
  set(_mj_source "MUJOCO_DIR")
endif()

# 2. Python package.
if(NOT _mj_found AND MJFORGE_MUJOCO_FROM_PYTHON AND NOT WIN32)
  find_package(Python3 COMPONENTS Interpreter QUIET)
  if(Python3_Interpreter_FOUND)
    execute_process(
      COMMAND "${Python3_EXECUTABLE}" -c "import mujoco, os; print(os.path.dirname(mujoco.__file__))"
      OUTPUT_VARIABLE _mj_py_dir OUTPUT_STRIP_TRAILING_WHITESPACE
      RESULT_VARIABLE _mj_py_res ERROR_QUIET)
    if(_mj_py_res EQUAL 0 AND _mj_py_dir)
      _mjforge_probe_mujoco("${_mj_py_dir}" _mj_found _mj_inc _mj_lib _mj_implib)
      if(_mj_found)
        set(_mj_source "Python package (${Python3_EXECUTABLE})")
      endif()
    endif()
  endif()
endif()

# 3. Official release download.
if(NOT _mj_found)
  set(_ver "${MJFORGE_MUJOCO_VERSION}")
  set(_base "https://github.com/google-deepmind/mujoco/releases/download/${_ver}")
  if(WIN32)
    set(_url "${_base}/mujoco-${_ver}-windows-x86_64.zip")
  elseif(APPLE)
    message(FATAL_ERROR
      "MuJoCo was not found. On macOS install it with 'pip install mujoco' (the editor will use the "
      "library inside the Python package) or pass -DMUJOCO_DIR=<folder containing include/ and libmujoco>.")
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(_url "${_base}/mujoco-${_ver}-linux-aarch64.tar.gz")
  else()
    set(_url "${_base}/mujoco-${_ver}-linux-x86_64.tar.gz")
  endif()
  message(STATUS "MJForge: downloading MuJoCo ${_ver} from ${_url}")
  include(FetchContent)
  FetchContent_Declare(mujoco_release URL "${_url}" DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  FetchContent_MakeAvailable(mujoco_release)
  _mjforge_probe_mujoco("${mujoco_release_SOURCE_DIR}" _mj_found _mj_inc _mj_lib _mj_implib)
  if(NOT _mj_found)
    message(FATAL_ERROR "Downloaded MuJoCo release has an unexpected layout: ${mujoco_release_SOURCE_DIR}")
  endif()
  set(_mj_source "downloaded release ${_ver}")
endif()

# Version from the header.
file(STRINGS "${_mj_inc}/mujoco/mujoco.h" _mj_ver_line REGEX "^#define mjVERSION_HEADER")
string(REGEX MATCH "[0-9]+" _mj_ver_num "${_mj_ver_line}")
math(EXPR _mj_major "${_mj_ver_num} / 1000000")
math(EXPR _mj_minor "(${_mj_ver_num} / 1000) % 1000")
math(EXPR _mj_patch "${_mj_ver_num} % 1000")
set(MJFORGE_MUJOCO_VERSION_FOUND "${_mj_major}.${_mj_minor}.${_mj_patch}")
if(_mj_ver_num LESS 3016000)
  message(WARNING "MJForge is developed against MuJoCo 3.16; found ${MJFORGE_MUJOCO_VERSION_FOUND}. Some features may not compile.")
endif()

add_library(mujoco::mujoco SHARED IMPORTED GLOBAL)
set_target_properties(mujoco::mujoco PROPERTIES
  IMPORTED_LOCATION "${_mj_lib}"
  INTERFACE_INCLUDE_DIRECTORIES "${_mj_inc}")
if(WIN32)
  set_target_properties(mujoco::mujoco PROPERTIES IMPORTED_IMPLIB "${_mj_implib}")
else()
  # The Python package ships an unversioned SONAME-less file name (libmujoco.so.X.Y.Z);
  # make the linker record exactly that file name.
  get_filename_component(_mj_soname "${_mj_lib}" NAME)
  set_target_properties(mujoco::mujoco PROPERTIES IMPORTED_SONAME "${_mj_soname}")
endif()
set(MJFORGE_MUJOCO_RUNTIME_FILE "${_mj_lib}")
message(STATUS "MJForge: MuJoCo ${MJFORGE_MUJOCO_VERSION_FOUND} from ${_mj_source}: ${_mj_lib}")
