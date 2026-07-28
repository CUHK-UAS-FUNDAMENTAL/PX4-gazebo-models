cmake_minimum_required(VERSION 3.16)
project(imav2026_gz_plugins)

# -----------------------------------------------------------------
# Gazebo Sim (Harmonic) — version determined at configure time
# -----------------------------------------------------------------
find_package(gz-sim7 REQUIRED)

set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

# -----------------------------------------------------------------
# Library: Imav2026LandingPlatformMotion
# -----------------------------------------------------------------
add_library(Imav2026LandingPlatformMotion
  SHARED
  Imav2026LandingPlatformMotion.cc
)

target_link_libraries(Imav2026LandingPlatformMotion
  PUBLIC
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)

target_include_directories(Imav2026LandingPlatformMotion
  PUBLIC
    ${gz-sim${GZ_SIM_VER}_INCLUDE_DIRS}
)

# -----------------------------------------------------------------
# Install into the standard Gazebo Sim plugin lookup directory so
# that `gz sim` finds the plugin automatically at runtime.
# -----------------------------------------------------------------
install(TARGETS Imav2026LandingPlatformMotion
  DESTINATION lib/gz-sim-${GZ_SIM_VER}/plugins
)
