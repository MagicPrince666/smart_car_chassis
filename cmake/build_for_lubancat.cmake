######## cross compile env define ###################
SET(CMAKE_SYSTEM_NAME Linux)
# 配置库的安装路径
SET(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install)

SET(CMAKE_SYSTEM_PROCESSOR aarch64)
# 工具链地址
# export STAGING_DIR=/home/yons/devel/LubanCatWrt/staging_dir
SET(TOOLCHAIN_DIR  "/home/yons/devel/LubanCatWrt/staging_dir/toolchain-aarch64_generic_gcc-11.3.0_musl/bin/")

include_directories(
    ${TOOLCHAIN_DIR}../../target-aarch64_generic_musl/usr/include
)

link_directories(
    ${TOOLCHAIN_DIR}../../target-aarch64_generic_musl/usr/lib
)

# rockchip rk3566
SET(CMAKE_C_COMPILER ${TOOLCHAIN_DIR}aarch64-openwrt-linux-musl-gcc)
SET(CMAKE_CXX_COMPILER ${TOOLCHAIN_DIR}aarch64-openwrt-linux-musl-g++)
