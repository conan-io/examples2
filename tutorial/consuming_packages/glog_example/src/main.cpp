#include <iostream>
#include <glog/logging.h>

int main(int argc, char* argv[])
{
    FLAGS_log_dir = "./"; // Write to the current directory
    google::InitGoogleLogging(argv[0]);
    LOG(INFO) << "Hello, World!" << std::endl;
    return 0;
}