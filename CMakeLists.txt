cmake_minimum_required(VERSION 3.16)
project(quadrf_mimo C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_C_FLAGS_RELEASE "-O2")
add_compile_options(-Wall -Wextra -Wno-unused-parameter)
# std::complex multiply without the C99 Annex G inf/NaN recovery call (__muldc3)
add_compile_options(-fcx-limited-range)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    # Pi 5 (Cortex-A76)
    add_compile_options(-mcpu=cortex-a76 -mtune=cortex-a76)
endif()

find_package(Threads REQUIRED)
include(GNUInstallDirs)

add_library(qmcore STATIC src/frame.cpp src/rx.cpp)
target_include_directories(qmcore PUBLIC src)
target_link_libraries(qmcore Threads::Threads m)

add_executable(qmtool src/qmtool.cpp)
target_link_libraries(qmtool qmcore)

add_executable(quadrf-mimo
    src/main.cpp
    src/hw.cpp
    src/link.cpp
    src/server.cpp
    external/mongoose.c
)
target_include_directories(quadrf-mimo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/external
)
target_compile_definitions(quadrf-mimo PRIVATE
    QM_WEB_DIR="${CMAKE_INSTALL_FULL_DATADIR}/quadrf/mimo"
)
set_source_files_properties(external/mongoose.c PROPERTIES COMPILE_OPTIONS "-w")
target_link_libraries(quadrf-mimo qmcore)

install(TARGETS quadrf-mimo DESTINATION ${CMAKE_INSTALL_BINDIR})
install(PROGRAMS packaging/quadrf-mimo-launch DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY web/ DESTINATION ${CMAKE_INSTALL_DATADIR}/quadrf/mimo)
install(FILES packaging/quadrf-mimo.json DESTINATION ${CMAKE_INSTALL_DATADIR}/quadrf/apps.d)
install(FILES packaging/com.scalerf.QuadRF.MIMO.desktop DESTINATION ${CMAKE_INSTALL_DATADIR}/applications)
install(FILES packaging/quadrf-mimo.svg DESTINATION ${CMAKE_INSTALL_DATADIR}/icons/hicolor/scalable/apps)
install(FILES packaging/com.scalerf.QuadRF.MIMO.metainfo.xml DESTINATION ${CMAKE_INSTALL_DATADIR}/metainfo)
install(FILES packaging/quadrf-mimo.1 DESTINATION ${CMAKE_INSTALL_MANDIR}/man1)
install(PROGRAMS packaging/46-mimo DESTINATION lib/quadrf/apply.d)
