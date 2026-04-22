#pragma once

#include <string>
#include <vector>

#include <tie_robot_msgs/PointCoords.h>
#include <tf2/LinearMath/Transform.h>

namespace tie_robot_process {
namespace planning {

struct CabinPoint
{
    float x = 0.0f;
    float y = 0.0f;
};

struct PseudoSlamBindGroup
{
    int group_index = 0;
    std::string group_type;
    std::vector<tie_robot_msgs::PointCoords> bind_points_world;
};

struct PseudoSlamGroupedAreaEntry
{
    int area_index = 0;
    CabinPoint cabin_point{};
    float cabin_z = 0.0f;
    std::vector<PseudoSlamBindGroup> bind_groups;
};

struct BindExecutionPathOriginPose
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct DynamicBindPlannerConfig
{
    float tcp_max_x_mm = 360.0f;
    float tcp_max_y_mm = 320.0f;
    float tcp_max_z_mm = 140.0f;
    float bind_execution_cabin_min_z_mm = 485.0f;
    float template_center_x_mm = 150.0f;
    float template_center_y_mm = 150.0f;
    float template_center_z_mm = 70.0f;
    float matrix_row_threshold_mm = 40.0f;
    float matrix_column_threshold_mm = 45.0f;
    float snake_row_tolerance_mm = 90.0f;
    int seed_neighbor_count = 8;
};

std::vector<PseudoSlamGroupedAreaEntry> build_dynamic_bind_area_entries_from_scan_world(
    const std::vector<tie_robot_msgs::PointCoords>& planning_world_points,
    const CabinPoint& path_origin,
    float cabin_height,
    const tf2::Transform& gripper_from_scepter,
    const DynamicBindPlannerConfig& config = DynamicBindPlannerConfig{});

BindExecutionPathOriginPose build_dynamic_bind_execution_path_origin(
    const std::vector<PseudoSlamGroupedAreaEntry>& bind_area_entries,
    const CabinPoint& planning_reference_origin,
    float cabin_height,
    float bind_execution_cabin_min_z_mm);

void sort_bind_area_entries_by_snake_rows(
    std::vector<PseudoSlamGroupedAreaEntry>& bind_area_entries,
    float row_tolerance_mm);

}  // namespace planning
}  // namespace tie_robot_process
