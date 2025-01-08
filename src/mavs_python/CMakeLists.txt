if (WIN32)
	file(WRITE mavs_python_paths.py "mavs_lib_path='${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}/mavs.dll'\nmavs_data_path='${CMAKE_SOURCE_DIR}/data'")
	file(WRITE mavs_gui.cmd "python ${CMAKE_SOURCE_DIR}/src/mavs_python/gui/sliderbar_sim.py  ${CMAKE_SOURCE_DIR}/src/mavs_python &")
	install(FILES mavs_gui.cmd DESTINATION bin
		PERMISSIONS OWNER_READ OWNER_EXECUTE OWNER_WRITE GROUP_EXECUTE)
else()
	file(WRITE mavs_python_paths.py "mavs_lib_path='${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/libmavs.so'\nmavs_data_path='${CMAKE_SOURCE_DIR}/data'")
	file(WRITE mavs_gui.sh "#!/bin/bash \n python3 ${CMAKE_SOURCE_DIR}/src/mavs_python/gui/sliderbar_sim.py  ${CMAKE_SOURCE_DIR}/src/mavs_python &")
	install(FILES mavs_gui.sh DESTINATION bin
		PERMISSIONS OWNER_READ OWNER_EXECUTE OWNER_WRITE GROUP_EXECUTE)
endif()
