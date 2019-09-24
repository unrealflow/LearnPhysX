cmake_minimum_required(VERSION 2.8)
project(LearnPhysX)

if (MSVC_VERSION GREATER_EQUAL "1900")
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("/std:c++latest" _cpp_latest_flag_supported)
    if (_cpp_latest_flag_supported)
        add_compile_options("/std:c++latest")
    endif()
endif()

set(SK_PROJECT_DIR ${PROJECT_SOURCE_DIR})
set(SK_INCLUDE_DIR 
    ${SK_PROJECT_DIR}/include 
    ${SK_PROJECT_DIR}/base
    "E:/VulkanSDK/PhysX/physx/include"
    "E:/VulkanSDK/PhysX/pxshared/include"
    "E:/VulkanSDK/PhysX/physx/snippets/graphics/include/win32/GL"
    "E:/VulkanSDK/PhysX/physx/source/common/src"
    "E:/VulkanSDK/PhysX/physx/source/foundation/include"
)
link_directories(
    ${SK_PROJECT_DIR}/lib 
    "E:/VulkanSDK/PhysX/physx/snippets/Graphics/lib/win64"
    "E:/VulkanSDK/PhysX/physx/snippets/Graphics/lib/win64/glut"
    "E:/VulkanSDK/PhysX/physx/source/common/src"
)

add_definitions(-DRENDER_SNIPPET=1)
include_directories(${SK_INCLUDE_DIR})

IF (CMAKE_BUILD_TYPE STREQUAL Debug)       
    link_directories("E:/VulkanSDK/PhysX/physx/bin/win.x86_64.vc142.mt/debug")
    add_compile_options(/MTd)
ELSE()
    link_directories("E:/VulkanSDK/PhysX/physx/bin/win.x86_64.vc142.mt/release")
    add_compile_options(/MT)
ENDIF()

link_libraries(
    PhysXExtensions_static_64.lib
    PhysX_64.lib
    PhysXPvdSDK_static_64.lib
    PhysXVehicle_static_64.lib
    PhysXCharacterKinematic_static_64.lib
    PhysXCooking_64.lib
    PhysXCommon_64.lib
    SnippetUtils_static_64.lib
    SnippetRender_static_64.lib
    PhysXFoundation_64.lib
)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${SK_PROJECT_DIR}/bin)   
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${SK_PROJECT_DIR}/bin)   
add_subdirectory(${SK_PROJECT_DIR}/base)
link_libraries(base)
add_subdirectory(${SK_PROJECT_DIR}/src)