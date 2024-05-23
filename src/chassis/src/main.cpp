#include "chassis/chassis_service.h"
#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
#include <ros/ros.h>
#else
#include <rclcpp/rclcpp.hpp>
#endif

#ifdef BACKTRACE_DEBUG
#include <execinfo.h>

#define PRINT_SIZE_ 100

static void _signal_handler(int signum)
{
    void *array[PRINT_SIZE_];
    char **strings;

    size_t size = backtrace(array, PRINT_SIZE_);
    strings     = backtrace_symbols(array, size);

    if (strings == nullptr) {
        fprintf(stderr, "backtrace_symbols");
        exit(EXIT_FAILURE);
    }

    switch (signum) {
    case SIGSEGV:
        fprintf(stderr, "widebright received SIGSEGV! Stack trace:\n");
        break;

    case SIGPIPE:
        fprintf(stderr, "widebright received SIGPIPE! Stack trace:\n");
        break;

    case SIGFPE:
        fprintf(stderr, "widebright received SIGFPE! Stack trace:\n");
        break;

    case SIGABRT:
        fprintf(stderr, "widebright received SIGABRT! Stack trace:\n");
        break;

    default:
        break;
    }

    for (size_t i = 0; i < size; i++) {
        fprintf(stderr, "%d %s \n", i, strings[i]);
    }

    free(strings);
    signal(signum, SIG_DFL); /* 还原默认的信号处理handler */

    exit(1);
}
#endif

int main(int argc, char *argv[])
{
#ifdef BACKTRACE_DEBUG
    signal(SIGPIPE, _signal_handler); // SIGPIPE，管道破裂。
    signal(SIGSEGV, _signal_handler); // SIGSEGV，非法内存访问
    signal(SIGFPE, _signal_handler);  // SIGFPE，数学相关的异常，如被0除，浮点溢出，等等
    signal(SIGABRT, _signal_handler); // SIGABRT，由调用abort函数产生，进程非正常退出
#endif

#if defined(USE_ROS_NORTIC_VERSION) || defined(USE_ROS_MELODIC_VERSION)
    ros::init(argc, argv, "chassis");
    auto ros_node = std::make_shared<ros::NodeHandle>();
    auto chassis = std::make_shared<ChassisSrv>(ros_node);

    ros::MultiThreadedSpinner spinner(std::thread::hardware_concurrency());
    spinner.spin();
    ros::shutdown();
#else
    rclcpp::init(argc, argv);

    // 多线程执行器
#if defined(USE_GALACTIC_VERSION) || defined(USE_HUMBLE_VERSION) || defined(USE_IRON_VERSION)
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), std::thread::hardware_concurrency(), true);
#else
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(), std::thread::hardware_concurrency(), true);
#endif
    auto node = std::make_shared<rclcpp::Node>("chassis");
    std::unique_ptr<ChassisSrv> chassis(new ChassisSrv(node));
    executor.add_node(node);
    RCLCPP_INFO(node->get_logger(), "threads = %ld", executor.get_number_of_threads());
    executor.spin();
    rclcpp::shutdown();
#endif

    return 0;
}
