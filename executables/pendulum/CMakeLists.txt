file(GLOB EXECUTABLES_PENDULUM_FILES
	*.cpp)

set(EXECUTABLE_OUTPUT_PATH ${EXECUTABLE_OUTPUT_PATH}/pendulum )

	
foreach(f ${EXECUTABLES_PENDULUM_FILES})
	GET_FILENAME_COMPONENT(F_EXEC ${f} NAME_WE )
	add_executable(${F_EXEC} ${f})
	target_link_directories(${F_EXEC} PUBLIC
    ${OpenCV_LIB_DIRS}
    )	
	target_link_libraries(${F_EXEC} 
	    ${OpenCV_LIBS} ${realsense2_LIBRARY} glfw
    OpenGL::GL
    )
	target_include_directories(${F_EXEC} PUBLIC
    ${OpenCV_INCLUDE_DIRS} 
    )
	target_link_libraries(${F_EXEC}
		)

endforeach()
