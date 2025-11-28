cmake_minimum_required(VERSION 3.16)
project(PointCloudCompress)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 编译器特定设置
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
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

# 安装目标（可选）
install(TARGETS pcd_compress DESTINATION bin)

# 测试（可选）
enable_testing()
add_test(NAME basic_test COMMAND pcd_compress --help)

# 打包（可选）
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)