px4_add_library(tinympc
	tinympc/src/tinympc/admm.cpp
	tinympc/src/tinympc/tiny_api.cpp)
target_include_directories(tinympc PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/tinympc/include/Eigen)
