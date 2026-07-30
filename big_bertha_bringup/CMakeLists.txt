cmake_minimum_required(VERSION 3.8)
project(big_bertha_bringup)

find_package(ament_cmake REQUIRED)

# Hardware bringup stub. launch/ and config/ are empty placeholders for now
# (each keeps a .gitkeep); the real servo bridge + sensor drivers + hardware
# launch files land here when the physical Big Bertha is wired up. The BOM and
# the sim->hardware mapping are documented in README.md (see PLAN.md sec 11).
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
