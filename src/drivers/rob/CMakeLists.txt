AUX_SOURCE_DIRECTORY(. SOURCE_LIST_ROB_MOTORS)
add_library(rob_motors
        STATIC
        ${SOURCE_LIST_ROB_MOTORS}
)
target_include_directories(rob_motors PUBLIC ./ ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(rob_motors PUBLIC ${PUBLIC_DEPENDENCIES} motors_protocol)