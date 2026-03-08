#include "grc26/controller_config.hpp"

ControllerConfig::ControllerConfig()
  : controllers_{
      std::array<PID, 6>{
        PID(1.0, 0.0, 0.0),
        PID(1.0, 0.0, 0.0),
        PID(1.0, 0.0, 0.0),
        PID(1.0, 0.0, 0.0),
        PID(1.0, 0.0, 0.0),
        PID(1.0, 0.0, 0.0)
      }
    }
{
  for (int i = 0; i < 6; ++i)
  {
    controllers_.cart_ctrl[i] = PID(1.0, 0.0, 0.0);
  }
}

bool ControllerConfig::load(const std::string& file)
{
  try
  {
    const std::string path =
        ament_index_cpp::get_package_share_directory("grc26") +
        "/config/" + file;

    YAML::Node root = YAML::LoadFile(path);

    auto cartesian_lin_ctrl = root["linear_vel_pid"];
    if (cartesian_lin_ctrl)
    {
      for (int i = 0; i < 3; ++i)
      {
        const std::string axis = (i == 0 ? "x" : (i == 1 ? "y" : "z"));
        const YAML::Node node = cartesian_lin_ctrl[axis];
        if (node)
        {
          controllers_.cart_ctrl[i].set_params(
              node["kp"] ? node["kp"].as<double>() : controllers_.cart_ctrl[i].kp,
              node["ki"] ? node["ki"].as<double>() : controllers_.cart_ctrl[i].ki,
              node["kd"] ? node["kd"].as<double>() : controllers_.cart_ctrl[i].kd,
              node["error_sum_tol"] ? node["error_sum_tol"].as<double>()       : controllers_.cart_ctrl[i].err_sum_tol,
              node["decay_rate"] ? node["decay_rate"].as<double>()             : controllers_.cart_ctrl[i].decay_rate,
              node["dead_zone_limit"] ? node["dead_zone_limit"].as<double>()   : controllers_.cart_ctrl[i].dead_zone_limit,
              node["lp_filter_alpha"] ? node["lp_filter_alpha"].as<double>()   : controllers_.cart_ctrl[i].lp_filter_alpha,
              node["saturation_limit"] ? node["saturation_limit"].as<double>() : controllers_.cart_ctrl[i].saturation_limit);
        }
      }
    }

    auto cartesian_ang_ctrl = root["orientation_ctrl"];
    if (cartesian_ang_ctrl)
    {
      double j = 0.0;
      for (int i = 0; i < 3; ++i)
      {
        const std::string axis = (i == 0 ? "x" : (i == 1 ? "y" : "z"));
        const YAML::Node node = cartesian_ang_ctrl[axis];
        if (node)
        {
          j = i + 3;
          controllers_.cart_ctrl[j].set_params(
            node["kp"] ? node["kp"].as<double>() : controllers_.cart_ctrl[j].kp,
            node["ki"] ? node["ki"].as<double>() : controllers_.cart_ctrl[j].ki,
            node["kd"] ? node["kd"].as<double>() : controllers_.cart_ctrl[j].kd,
            node["error_sum_tol"] ? node["error_sum_tol"].as<double>()       : controllers_.cart_ctrl[j].err_sum_tol,
            node["decay_rate"] ? node["decay_rate"].as<double>()             : controllers_.cart_ctrl[j].decay_rate,
            node["dead_zone_limit"] ? node["dead_zone_limit"].as<double>()   : controllers_.cart_ctrl[j].dead_zone_limit,
            node["lp_filter_alpha"] ? node["lp_filter_alpha"].as<double>()   : controllers_.cart_ctrl[j].lp_filter_alpha,
            node["saturation_limit"] ? node["saturation_limit"].as<double>() : controllers_.cart_ctrl[j].saturation_limit);
        }
      }
    }
    return true;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Failed to load controller config: "
              << e.what() << std::endl;
    return false;
  }
}