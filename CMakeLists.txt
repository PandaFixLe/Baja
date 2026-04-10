# 1. 声明 CMake 最低版本和项目名称
cmake_minimum_required(VERSION 3.10)
project(baja_controller VERSION 1.0 LANGUAGES CXX)

# 2. 设置 C++ 标准（现代 C++ 必选）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 3. 设置头文件搜索路径（让 #include "xxx.h" 能找到）
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# 4. 收集源文件（两种方式）
# 方式 A：手动列出（推荐，明确可控）
set(SOURCES
    src/main.cpp
    src/pid_controller.cpp
    src/pure_pursuit_controller.cpp
)

# 5. 添加可执行目标
add_executable(controller ${SOURCES})

# 6. 编译选项（工程规范）
target_compile_options(controller PRIVATE
    -Wall      # 开启所有警告
    -Wextra    # 额外警告
    -pedantic  # 严格遵循标准
)

# 7. 调试/发布配置（可选但推荐）
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_definitions(controller PRIVATE DEBUG_MODE)
    message(STATUS "🔧 编译模式: Debug")
else()
    target_compile_options(controller PRIVATE -O2)  # 发布版优化
    message(STATUS "🚀 编译模式: Release")
endif()

# 8. 输出信息（方便确认）
message(STATUS "✅ 项目配置完成: ${PROJECT_NAME} v${PROJECT_VERSION}")