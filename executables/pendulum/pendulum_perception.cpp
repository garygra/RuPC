#if __linux__
// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <Eigen/Core>
#include <Eigen/Dense>

#include <librealsense2/rs.hpp>  // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>    // Include OpenCV API

#include <fstream>
#include <vector>

double detect_pendulum(cv::Mat& img, int w, int h)
{
  std::vector<cv::Point> locations;  // output, locations of non-zero pixels
  cv::findNonZero(img, locations);

  Eigen::MatrixXd A{ locations.size(), 2 };
  Eigen::VectorXd b{ locations.size() };
  double x_accum{ 0 };
  double y_accum{ 0 };
  for (auto j = 0; j < locations.size(); ++j)
  {
    A(j, 0) = locations[j].y;
    A(j, 1) = 1;
    b(j) = locations[j].x;
    x_accum += locations[j].y;
    y_accum += locations[j].x;
  }
  x_accum = x_accum / locations.size();
  y_accum = y_accum / locations.size();

  Eigen::Vector2d x = (A.transpose() * A).ldlt().solve(A.transpose() * b);

  const double m{ x[0] };
  const double bd{ x[1] };

  const double x_img_center{ h / 2.0 };
  const double y_img_center{ w / 2.0 };

  auto f = [&](const double x) { return m * x + bd; };

  const double y1{ f(x_accum) };
  const double y2{ f(x_img_center) };

  const double x1{ x_accum };
  const double x2{ x_img_center };
  double theta = -M_PI / 2.0 + std::atan2(x2 - x1, y2 - y1);

  cv::Point p1(y1, x1), p2(y2, x2);

  cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
  cv::line(img, p1, p2, CV_RGB(255, 0, 0), 2, cv::LINE_8);
  cv::rectangle(img, cv::Point(10, 2), cv::Point(150, 20), CV_RGB(255, 255, 255), -1);
  cv::putText(img,                                              // target image
              std::to_string(theta),                            // text
              cv::Point(15, 15),                                // top-left position
              cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(255, 0, 0),  // font color
              1);
  circle(img, cv::Point(y_accum, x_accum), 10, CV_RGB(255, 0, 0), cv::FILLED, cv::LINE_8);

  return theta;
}

int main(int argc, char* argv[])
try
{
  std::cout << '\n' << "Press a key to continue...";
  do
  {
  } while (std::cin.get() != '\n');
  // Declare depth colorizer for pretty visualization of depth data
  rs2::colorizer color_map;

  // Declare RealSense pipeline, encapsulating the actual device and sensors
  // Start streaming with default recommended configuration

  int width = 640;
  int height = 480;
  int fps = 60;
  rs2::config config;
  config.enable_all_streams();
  config.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
  config.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);
  // config.enable_stream(RS2_STREAM_DEPTH, 0, width, height, RS2_FORMAT_Z16 ,
  // fps);

  rs2::pipeline pipe;
  auto pipe_prof = pipe.start(config);
  // auto color_sensor = pipe_prof.get_device().first<rs2::color_sensor>();
  // auto depth_sensor = pipe_prof.get_device().first<rs2::depth_sensor>();
  // color_sensor.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 0.f);

  for (auto&& s : pipe_prof.get_device().query_sensors())
  {
    s.set_option(RS2_OPTION_GLOBAL_TIME_ENABLED, 1.f);
  }

  using namespace cv;
  const auto color_window_name = "Color";
  const auto depth_window_name = "Depth";
  const auto infr1_window_name = "infr1";
  const auto infr2_window_name = "infr2";

  namedWindow(color_window_name, WINDOW_AUTOSIZE);
  namedWindow(depth_window_name, WINDOW_AUTOSIZE);
  namedWindow(infr1_window_name, WINDOW_AUTOSIZE);
  namedWindow(infr2_window_name, WINDOW_AUTOSIZE);

  bool first = true;
  //  cv::Mat bck_orig = cv::imread(
  //    "/home/pracsys/pendulum_perception/pendulum_background_Color.png",
  //  IMREAD_COLOR);
  // cv::Mat bck(Size(width,height), CV_8UC3);

  int old_color_frame_count = -1;
  int old_depth_frame_count = -1;
  int old_infr1_frame_count = -1;
  int old_infr2_frame_count = -1;

  std::ofstream ofs_map;
  ofs_map.open("measurements_3.txt", std::ofstream::trunc);
  std::cout << std::fixed << std::showpoint << std::setprecision(10);

  while (waitKey(1) < 0 && getWindowProperty(color_window_name, WND_PROP_AUTOSIZE) >= 0)
  {
    // rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of
    // frames from the camera rs2::frame depth =
    // data.get_depth_frame().apply_filter(color_map); rs2::frame rgb =
    // data.get_rgb_frame();
    rs2::frame new_color_frame;
    rs2::frame new_depth_frame;
    rs2::frame new_infr1_frame;
    rs2::frame new_infr2_frame;

    rs2::frameset fs;
    bool new_frames = pipe.poll_for_frames(&fs);
    if (new_frames)
    {
      // for (const rs2::frame& f : fs)
      new_color_frame = fs.get_color_frame();
      new_depth_frame = fs.get_depth_frame();
      new_infr1_frame = fs.get_infrared_frame(1);
      new_infr2_frame = fs.get_infrared_frame(2);
    }

    if (new_color_frame)
    {
      auto color_fc = new_color_frame.get_frame_metadata(RS2_FRAME_METADATA_FRAME_COUNTER);
      if (color_fc > old_color_frame_count)
      {
        old_color_frame_count = color_fc;
        // RS2_FRAME_METADATA_SENSOR_TIMESTAMP:
        // 									Timestamp of the
        // middle of sensor's exposure calculated by device. usec
        auto color_timestamp = new_color_frame.get_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP);
        ofs_map << "color " << color_fc << " " << color_timestamp;

        const int w = new_color_frame.as<rs2::video_frame>().get_width();
        const int h = new_color_frame.as<rs2::video_frame>().get_height();

        // Create OpenCV matrix of size (w,h) from the colorized depth data
        cv::Mat img(Size(w, h), CV_8UC3, (void*)new_color_frame.get_data(), Mat::AUTO_STEP);
        // 640 x 480
        // cv::Mat cropped = img(Range(100,600), Range(100,400));
        cv::Mat bck{ img.size(), img.type() };
        // cv::resize(bck_orig, bck, img.size(), cv::INTER_LINEAR);

        // cv::cvtColor(bck, bck, cv::COLOR_BGR2RGB);
        // cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
        cvtColor(bck, bck, cv::COLOR_RGBA2GRAY);
        cvtColor(img, img, cv::COLOR_BGRA2GRAY);
        img = img - bck;

        cv::threshold(img, img, 100, 255, cv::THRESH_BINARY);
        double theta = detect_pendulum(img, w, h);

        ofs_map << " " << theta << "\n";

        imshow(color_window_name, img);
      }
    }

    if (new_depth_frame)
    {
      auto depth_fc = new_color_frame.get_frame_metadata(RS2_FRAME_METADATA_FRAME_COUNTER);
      if (depth_fc > old_depth_frame_count)
      {
        old_depth_frame_count = depth_fc;
        auto depth_timestamp = new_depth_frame.get_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP);
        ofs_map << "depth " << depth_fc << " " << depth_timestamp;

        const int w = new_depth_frame.as<rs2::video_frame>().get_width();
        const int h = new_depth_frame.as<rs2::video_frame>().get_height();

        // if (first) std::cout << "w: " << w << "\th: " << h <<std::endl;

        cv::Mat img_full(Size(w, h), CV_16UC1, (void*)new_depth_frame.get_data(), Mat::AUTO_STEP);

        cv::Mat img = img_full(cv::Rect(0, 0, w, h - 50));
        // cv::(img_full, img, Size(w,h), cv::INTER_LINEAR);
        cv::convertScaleAbs(img, img, 0.5, 1);
        cv::Mat img_cm;
        cv::applyColorMap(img, img, cv::COLORMAP_JET);
        cv::threshold(img, img, 200, 255, cv::THRESH_BINARY);
        cvtColor(img, img, cv::COLOR_BGRA2GRAY);

        double theta = detect_pendulum(img, w, h);

        ofs_map << " " << theta << "\n";

        imshow(depth_window_name, img);
      }
    }

    if (new_infr1_frame)
    {
      auto infr1_fc = new_infr1_frame.get_frame_metadata(RS2_FRAME_METADATA_FRAME_COUNTER);
      if (infr1_fc > old_infr1_frame_count)
      {
        old_infr1_frame_count = infr1_fc;
        auto infr1_timestamp = new_infr1_frame.get_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP);
        ofs_map << "infr1 " << infr1_fc << " " << infr1_timestamp;

        const int w = new_infr1_frame.as<rs2::video_frame>().get_width();
        const int h = new_infr1_frame.as<rs2::video_frame>().get_height();

        cv::Mat img_full(Size(w, h), CV_8UC1, (void*)new_infr1_frame.get_data(), Mat::AUTO_STEP);

        cv::Mat img = img_full(cv::Rect(200, 40, w - 400, h - 50));
        imshow(infr1_window_name, img);
        cv::GaussianBlur(img, img, cv::Size(15, 15), 0);
        cv::threshold(img, img, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);

        double theta = detect_pendulum(img, w, h);

        ofs_map << " " << theta << "\n";
        // imshow(infr1_window_name, img);
      }
    }

    if (new_infr2_frame)
    {
      auto infr2_fc = new_infr2_frame.get_frame_metadata(RS2_FRAME_METADATA_FRAME_COUNTER);
      if (infr2_fc > old_infr2_frame_count)
      {
        old_infr2_frame_count = infr2_fc;
        auto infr2_timestamp = new_infr2_frame.get_frame_metadata(RS2_FRAME_METADATA_SENSOR_TIMESTAMP);
        ofs_map << "infr2 " << infr2_fc << " " << infr2_timestamp;

        const int w = new_infr2_frame.as<rs2::video_frame>().get_width();
        const int h = new_infr2_frame.as<rs2::video_frame>().get_height();

        cv::Mat img_full(Size(w, h), CV_8UC1, (void*)new_infr2_frame.get_data(), Mat::AUTO_STEP);

        cv::Mat img = img_full(cv::Rect(175, 40, w - 425, h - 50));

        cv::GaussianBlur(img, img, cv::Size(15, 15), 0);
        cv::threshold(img, img, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);

        double theta = detect_pendulum(img, w, h);

        ofs_map << " " << theta << "\n";

        imshow(infr2_window_name, img);
      }
    }
    first = false;
  }

  ofs_map.close();
  return EXIT_SUCCESS;
}
catch (const rs2::error& e)
{
  std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    "
            << e.what() << std::endl;
  return EXIT_FAILURE;
}
catch (const std::exception& e)
{
  std::cerr << e.what() << std::endl;
  return EXIT_FAILURE;
}

#elif __APPLE__
#include <cstdio>
int main()
{
  printf("%s unsupported in MacOS\n", __FILE__);
}
#else
#error "Unknown compiler"
#endif