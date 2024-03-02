######## cross compile env define ###################
SET(CMAKE_SYSTEM_NAME Linux)
# 配置库的安装路径
SET(CMAKE_INSTALL_PREFIX ${CMAKE_BINARY_DIR}/install)

SET(CMAKE_SYSTEM_PROCESSOR riscv64)
# 工具链地址
SET(TOOLCHAIN_DIR  "/home/prince/Tina-Linux/lichee/brandy-2.0/tools/toolchain/riscv64-linux-x86_64-20200528/bin/")

include_directories(
#    "${CMAKE_SOURCE_DIR}/thiryparty/faac/include"
    "/home/prince/Tina-Linux/out/d1-mq_pro/staging_dir/target/usr/include"
)

link_directories(
#    "${CMAKE_SOURCE_DIR}/thiryparty/faac/lib"
    "/home/prince/Tina-Linux/out/d1-mq_pro/staging_dir/target/usr/lib/"
    ${CMAKE_SOURCE_DIR}
)

# sunxi D1
SET(CMAKE_C_COMPILER ${TOOLCHAIN_DIR}riscv64-unknown-linux-gnu-gcc)
SET(CMAKE_CXX_COMPILER ${TOOLCHAIN_DIR}riscv64-unknown-linux-gnu-g++)
