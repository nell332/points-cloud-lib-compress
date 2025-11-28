cmake_minimum_required(VERSION 3.16)
project(PointCloudCompress)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 编译器特定设置
if(MSVC)
    add_compile_options(/W4)
    add_compile_options("/utf-8")
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
    add_compile_options("-finput-charset=UTF-8" "-fexec-charset=UTF-8")
endif()

# 查找依赖包
find_package(PCL 1.12 REQUIRED COMPONENTS common io filters)
find_package(yaml-cpp REQUIRED)

# 包含目录
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIRS})

# 添加可执行文件
add_executable(pcd_compress
    pcd_compress.cpp
    PointCloudCompress.cpp
    public_tools.cpp
    TimeTool.cpp
)

# 链接库
target_link_libraries(pcd_compress
    ${PCL_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)

# Windows特定设置
if(WIN32)
    target_link_libraries(pcd_compress ws2_32)
endif()

# 设置输出目录为上级目录的bin文件夹
set(BIN_OUTPUT_DIR "${CMAKE_SOURCE_DIR}/bin")

# 创建bin目录（如果不存在）
add_custom_command(TARGET pcd_compress PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${BIN_OUTPUT_DIR}"
    COMMENT "Creating bin directory..."
)

# 复制可执行文件到上级bin目录
add_custom_target(copy_binary ALL
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:pcd_compress> "${BIN_OUTPUT_DIR}/"
    DEPENDS pcd_compress
    COMMENT "Copying executable to ${BIN_OUTPUT_DIR}"
)

# 安装目标（可选）
install(TARGETS pcd_compress DESTINATION bin)

# 安装pcd.yaml配置文件
if(EXISTS "${CMAKE_SOURCE_DIR}/pcd_compress.yaml")
    install(FILES pcd_compress.yaml DESTINATION bin)
    message(STATUS "Found pcd_compress.yaml, will install it to bin directory")
else()
    message(WARNING "pcd_compress.yaml not found in source directory, skipping installation")
endif()

# 测试（可选）
enable_testing()
add_test(NAME basic_test COMMAND pcd_compress --help)

# 打包（可选）
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)