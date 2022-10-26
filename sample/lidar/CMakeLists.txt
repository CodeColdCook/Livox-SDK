cmake_minimum_required(VERSION 3.0)

set(DEMO_NAME lidar_sample)
add_executable(${DEMO_NAME} main.c)

target_link_libraries(${DEMO_NAME}
        PRIVATE
        ${PROJECT_NAME}_static
        )

add_executable(test_set_mode test_set_mode.c)
target_link_libraries(test_set_mode
        PRIVATE
        ${PROJECT_NAME}_static
        )

add_executable(livox_mode_set livox_mode_set.c)
target_link_libraries(livox_mode_set
        PRIVATE
        ${PROJECT_NAME}_static
        )
