# Host simulator of the OpenSENT instrument (real app code, virtual hardware).
#   cmake -S firmware/sim -B build-sim && cmake --build build-sim && ./build-sim/opensent-sim
cmake_minimum_required(VERSION 3.13)
project(opensent_sim C)
set(CMAKE_C_STANDARD 99)
set(ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)
add_executable(opensent-sim sim.c ${ROOT}/firmware/app/app.c
  ${ROOT}/core/sent_common.c ${ROOT}/core/sent_rx.c ${ROOT}/core/sent_tx.c)
target_include_directories(opensent-sim PRIVATE ${ROOT}/core ${ROOT}/firmware/app)
target_compile_options(opensent-sim PRIVATE -Wall -Wextra -O2)
target_link_libraries(opensent-sim m)
