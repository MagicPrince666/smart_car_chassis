######## cross compile env define ###################
SET(CMAKE_SYSTEM_NAME Linux)
# 配置库的安装路径
SET(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install)

# export STAGING_DIR=/Volumes/unix/HandsomeMod/staging_dir
SET(CMAKE_SYSTEM_PROCESSOR aarch64)

# 工具链地址
SET(TOOLCHAIN_DIR  "/Volumes/unix/openwrt/staging_dir/toolchain-aarch64_generic_gcc-12.3.0_musl/")

include_directories(
    ${TOOLCHAIN_DIR}../../target-aarch64_generic_musl/usr/include
)

link_directories(
    ${TOOLCHAIN_DIR}../../target-aarch64_generic_musl/usr/lib
    ${CMAKE_SOURCE_DIR}
)

find_package(libX264 REQUIRED)

# toolchain
SET(CMAKE_C_COMPILER ${TOOLCHAIN_DIR}bin/aarch64-openwrt-linux-musl-gcc)
SET(CMAKE_CXX_COMPILER ${TOOLCHAIN_DIR}bin/aarch64-openwrt-linux-musl-g++)
