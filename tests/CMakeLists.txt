# Host-side unit tests for the portable SENT core.
#   cmake -S tests -B build-native && cmake --build build-native && ctest --test-dir build-native
cmake_minimum_required(VERSION 3.13)
project(opensent_native C)
set(CMAKE_C_STANDARD 99)
set(CORE ${CMAKE_CURRENT_LIST_DIR}/../core)
add_library(sentcore STATIC ${CORE}/sent_common.c ${CORE}/sent_rx.c ${CORE}/sent_tx.c)
target_include_directories(sentcore PUBLIC ${CORE})
target_compile_options(sentcore PRIVATE -Wall -Wextra -Werror -Wshadow -Wdouble-promotion -fno-common)
target_link_libraries(sentcore PUBLIC m)

add_executable(test_sent test_sent.c)
target_link_libraries(test_sent sentcore)
target_compile_options(test_sent PRIVATE -Wall -Wextra)

add_executable(sent_replay sent_replay.c)
target_link_libraries(sent_replay sentcore)

enable_testing()
add_test(NAME sent COMMAND test_sent ${CMAKE_CURRENT_LIST_DIR}/data)
