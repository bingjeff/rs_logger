#include <librealsense2/rs.hpp>
#include <chrono>
#include <iostream>
#include <string.h>
#include <thread>

rs2::config setup_depth_camera(const rs2::device& input_device, const std::string& file_counter) {
  rs2::config pipeline_config;
  pipeline_config.enable_record_to_file("/tmp/" + file_counter +"-depth.bag");
  std::string device_name = input_device.get_info(RS2_CAMERA_INFO_NAME);
  std::string serial_number = input_device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
  std::cout << "Enabling: " << device_name << " " << serial_number << "... ";
  pipeline_config.enable_device(serial_number);
  std::cout << "done.\n";
  return pipeline_config;
}

rs2::config setup_tracking_camera(const rs2::device& input_device, const std::string& file_counter) {
  rs2::config pipeline_config;
  pipeline_config.enable_record_to_file("/tmp/" + file_counter +"-pose.bag");
  pipeline_config.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
  std::string device_name = input_device.get_info(RS2_CAMERA_INFO_NAME);
  std::string serial_number = input_device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
  std::cout << "Enabling: " << device_name << " " << serial_number << "... ";
  pipeline_config.enable_device(serial_number);
  std::cout << "done.\n";
  return pipeline_config;
}

int main(int argc, char *argv[])
try
{
  std::cout << "Looking for devices...\n";
  rs2::context rs_context;
  auto device_list = rs_context.query_devices();

  // Get default device configurations for tracking and depth cameras.
  const std::string kCameraTypeD400 = "D400";
  const std::string kCameraTypeT200 = "T200";
  std::string file_counter = "000";
  std::vector<rs2::config> device_configurations;
  for (const auto& device : device_list) {
    if(device.get_info(RS2_CAMERA_INFO_PRODUCT_LINE) == kCameraTypeD400) {
      device_configurations.emplace_back(setup_depth_camera(device, file_counter));
    } 
    if (device.get_info(RS2_CAMERA_INFO_PRODUCT_LINE) == kCameraTypeT200) {
      device_configurations.emplace_back(setup_tracking_camera(device, file_counter));
    }
  }

  // Setup the pipelines to start logging.
  std::cout << "Starting pipelines... ";
  std::vector<rs2::pipeline> pipelines;
  for (auto& device_config : device_configurations) {
    rs2::pipeline pipe(rs_context);
    pipe.start(device_config);
    pipelines.emplace_back(pipe);
  }
  std::cout << "done.\n";

  // Log for a fixed amount of time.
  std::cout << "Logging: ";
  auto time_end = std::chrono::system_clock::now() + std::chrono::seconds(5);
  while (std::chrono::system_clock::now() < time_end) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    std::cout << ".";
  }
  std::cout << " done.\n";
  
  // Close the pipelines.
  std::cout << "Stopping pipelines... ";
  for (auto& pipe : pipelines) {
    pipe.stop();
  }
  std::cout << "done.\n";
  
  return EXIT_SUCCESS;
}
catch (const rs2::error &e)
{
  std::cerr << "RealSense error calling "
            << e.get_failed_function()
            << "(" << e.get_failed_args()
            << "):\n    " << e.what() << std::endl;
  return EXIT_FAILURE;
}
catch (const std::exception &e)
{
  std::cerr << e.what()
            << std::endl;
  return EXIT_FAILURE;
}
