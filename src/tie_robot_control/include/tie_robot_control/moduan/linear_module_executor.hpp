#ifndef TIE_ROBOT_CONTROL_MODUAN_LINEAR_MODULE_EXECUTOR_HPP
#define TIE_ROBOT_CONTROL_MODUAN_LINEAR_MODULE_EXECUTOR_HPP

#include <chrono>
#include <string>
#include <vector>

#include "tie_robot_hw/driver/linear_module_driver.hpp"
#include "tie_robot_msgs/PointCoords.h"

class ScopedModuleSpeedOverride
{
public:
    explicit ScopedModuleSpeedOverride(double override_speed);
    ~ScopedModuleSpeedOverride();

private:
    double original_speed_;
    bool active_;
};

void apply_module_speed_mm_per_sec(double new_speed);
std::string compose_linear_module_driver_error_message(
    const std::string& prefix,
    const tie_robot_hw::driver::DriverError& driver_error);
bool ensure_linear_module_driver_started(tie_robot_hw::driver::DriverError* error);
bool request_linear_module_zero_via_driver(std::string* error_message);
bool wait_linear_module_axis_arrival(int Axis, double target_coordinate);
bool arrive_z(int axis_z, double& z, bool& is_current_z);
void clear_finishall_flag_if_needed();
bool wait_for_plc_finish_all(std::chrono::milliseconds poll_interval, std::chrono::seconds timeout);
void moveLinearModule(double x, double y, double z, double angle);
int linear_module_move_origin_single(int Axis);
void move_linear_module_to_origin();
double max_bind_height_excess_mm(const std::vector<float>& out_of_height_z_values);
std::string append_bind_height_excess_message(const std::string& message, double height_excess_mm);
bool should_keep_jump_bind_point(const tie_robot_msgs::PointCoords& point);
bool is_valid_precomputed_tcp_travel_z(double local_z_mm);
void inputAllPoints(int i, double x, double y, double z, double rz);
bool execute_bind_points(
    const std::vector<tie_robot_msgs::PointCoords>& filteredPoints,
    std::string& response_message,
    bool apply_jump_bind_filter = true);

#endif
