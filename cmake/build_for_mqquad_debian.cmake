######## cross compile env define ###################
SET(CMAKE_SYSTEM_NAME Linux)
# 配置库的安装路径
SET(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install)

SET(CMAKE_SYSTEM_PROCESSOR aarch64)
# 工具链地址
SET(TOOLCHAIN_DIR  "/home/yons/devel/Allwinner-H616/toolchains/bin/")

# sunxi D1
SET(CMAKE_C_COMPILER ${TOOLCHAIN_DIR}aarch64-none-linux-gnu-gcc)
SET(CMAKE_CXX_COMPILER ${TOOLCHAIN_DIR}aarch64-none-linux-gnu-gnu-g++)
