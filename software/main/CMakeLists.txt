# # Sources
# file(GLOB_RECURSE srcs_c "*.c")
# list(APPEND srcs  ${srcs_c})

# file(GLOB_RECURSE srcs_cpp "*.cpp")
# list(APPEND srcs  ${srcs_cpp})


# Sources
set(srcs
            "main.cpp"
            "src/bluetooth_serial.cpp"
            "src/commands.cpp"
            "src/hw_motors_impl.cpp"
            "src/motors_on_esp32.cpp"
            "src/indication.cpp"
            "src/position_unit.cpp"
            "src/controlcallback.cpp"
            "src/serial.cpp"
            "src/zk_i2c.cpp"
            "src/registers.cpp")


# Includes
set(includes    "."
                "include")



#  Print
#  *********************************************************************
message("*******************************************************************************")
message("Project's info")

message("-- Sources")
foreach(el IN LISTS srcs)
message(" ${el}")
endforeach()

message("-- Includes")
foreach(el IN LISTS includes)
message(" ${el}")
endforeach()
message("*******************************************************************************")
# ********************************************************************** Print

idf_component_register( SRCS ${srcs}
                        INCLUDE_DIRS ${includes})
