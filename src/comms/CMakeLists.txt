file(GLOB_RECURSE RuPC_LIB_COMMS  ./*.cpp)

add_library(${PROJECT_NAME}_COMMS  ${RuPC_LIB_COMMS})

target_include_directories(${PROJECT_NAME}_COMMS PUBLIC ".")

