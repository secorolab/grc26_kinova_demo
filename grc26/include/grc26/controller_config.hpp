#ifndef CONTROLLER_CONFIG_HPP
#define CONTROLLER_CONFIG_HPP

#include <array>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include "grc26/pid_controller.hpp"
#include "grc26/stiffness_controller.hpp"

struct Controllers
{
    std::array<PID, 6> cart_ctrl = {
        PID(1.0, 0.0, 0.0),
        PID(1.0, 0.0, 0.0),
        PID(1.0, 0.0, 0.0),
        PID(1.0, 0.0, 0.0),
        PID(1.0, 0.0, 0.0),
        PID(1.0, 0.0, 0.0)
    };
};

class ControllerConfig
{
public:
    ControllerConfig();

    bool load(const std::string& config_path);
    const Controllers& controllers() const { return controllers_; }

private:
    Controllers controllers_;
};

#endif // CONTROLLER_CONFIG_HPP