# --- scene editor ----------------------------
add_executable(scene_editor scene_editor_main.cpp scene_editor.cpp )
target_link_libraries(scene_editor mavs_sim)
install(TARGETS scene_editor RUNTIME DESTINATION bin/utilities)