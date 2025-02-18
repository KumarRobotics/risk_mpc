//
// Created by xiang on 2021/10/8.
//
#include <gflags/gflags.h>
#include <unistd.h>
#include <csignal>

#include "laser_mapping.h"

/// run the lidar mapping in online mode

// DEFINE_string(traj_log_file, "./Log/traj.txt", "path to traj log file");
DEFINE_string(traj_log_file, (std::string(std::string(ROOT_DIR) + "Log/traj.txt")), "path to traj log file");

std::shared_ptr<faster_lio::LaserMapping> LM;  
std::thread finishThread;

void FinishTask() {
    LOG(INFO) << "Finishing mapping in a separate thread.";
    LM->Finish();
    LOG(INFO) << "Mapping finished.";
    faster_lio::Timer::PrintAll();
    LOG(INFO) << "save trajectory to: " << FLAGS_traj_log_file;
    LM->Savetrajectory(FLAGS_traj_log_file);
    LOG(INFO) << "Program exiting.";
}



void SigHandle(int sig) {
    faster_lio::options::FLAG_EXIT = true;
    // ROS_WARN("catch sig %d", sig);
    LOG(INFO) << "catch sig " << sig;
}

int main(int argc, char **argv) {
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(argv[0]);


    //ROS2
    rclcpp::init(argc, argv);
    LM = std::make_shared<faster_lio::LaserMapping>("laserMapping");
    // LM->InitROS();
    rclcpp::Rate rate(5000);
    RCLCPP_INFO(LM->get_logger(), "\033[1;32m---->\033[0m Started.");

    signal(SIGINT, SigHandle);

    // online, almost same with offline, just receive the messages from ros
    rclcpp::ExecutorOptions options;
    rclcpp::executors::MultiThreadedExecutor executor(options, 1);
    executor.add_node(LM);

    while(rclcpp::ok())
    {
        if(faster_lio::options::FLAG_EXIT)
        {
            executor.cancel();
            rclcpp::shutdown();
            break;
        }
        executor.spin_once();
        LM->Run();
        rate.sleep();
    }


    if (!finishThread.joinable()) {
        finishThread = std::thread(FinishTask);
    }

    if (finishThread.joinable()) {
        finishThread.join();
    }


    return 0;
}
