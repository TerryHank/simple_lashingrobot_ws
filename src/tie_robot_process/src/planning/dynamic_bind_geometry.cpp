#include "dynamic_bind_planning_internal.hpp"

#include <algorithm>
#include <cmath>

namespace tie_robot_process {
namespace planning {
namespace internal {

float clamp_bind_execution_cabin_z(float planned_cabin_z, const DynamicBindPlannerConfig& config)
{
    return std::max(planned_cabin_z, config.bind_execution_cabin_min_z_mm);
}

bool is_local_bind_point_in_range(
    const tie_robot_msgs::PointCoords& point,
    const DynamicBindPlannerConfig& config)
{
    return point.World_coord[0] >= 0.0f &&
           point.World_coord[0] <= config.tcp_max_x_mm &&
           point.World_coord[1] >= 0.0f &&
           point.World_coord[1] <= config.tcp_max_y_mm &&
           point.World_coord[2] >= 0.0f &&
           point.World_coord[2] <= config.tcp_max_z_mm;
}

void transform_cabin_world_point_to_planned_gripper_point(
    const tie_robot_msgs::PointCoords& world_point,
    const CabinPoint& cabin_point,
    float cabin_height,
    const tf2::Transform& gripper_from_scepter,
    tie_robot_msgs::PointCoords& gripper_point)
{
    const tf2::Vector3 point_in_scepter_frame(
        static_cast<double>(world_point.World_coord[0] - cabin_point.x) / 1000.0,
        static_cast<double>(world_point.World_coord[1] - cabin_point.y) / 1000.0,
        static_cast<double>(cabin_height - world_point.World_coord[2]) / 1000.0 -
            gripper_from_scepter.getOrigin().z());
    const tf2::Vector3 point_in_gripper_frame = gripper_from_scepter * point_in_scepter_frame;

    gripper_point = world_point;
    gripper_point.World_coord[0] = static_cast<float>(point_in_gripper_frame.x() * 1000.0);
    gripper_point.World_coord[1] = static_cast<float>(point_in_gripper_frame.y() * 1000.0);
    gripper_point.World_coord[2] = static_cast<float>(point_in_gripper_frame.z() * 1000.0);
}

double local_origin_distance_sq(const tie_robot_msgs::PointCoords& point)
{
    const double x = static_cast<double>(point.World_coord[0]);
    const double y = static_cast<double>(point.World_coord[1]);
    return x * x + y * y;
}

std::vector<PseudoSlamCandidatePoint> build_area_bind_candidates(
    const std::vector<tie_robot_msgs::PointCoords>& remaining_world_points,
    const CabinPoint& cabin_point,
    float cabin_height,
    const tf2::Transform& gripper_from_scepter,
    const DynamicBindPlannerConfig& config)
{
    std::vector<PseudoSlamCandidatePoint> candidates;
    for (size_t world_index = 0; world_index < remaining_world_points.size(); ++world_index) {
        tie_robot_msgs::PointCoords local_point;
        transform_cabin_world_point_to_planned_gripper_point(
            remaining_world_points[world_index],
            cabin_point,
            cabin_height,
            gripper_from_scepter,
            local_point);
        if (!is_local_bind_point_in_range(local_point, config)) {
            continue;
        }
        candidates.push_back({remaining_world_points[world_index], local_point, world_index});
    }
    return candidates;
}

DynamicBindPlanningCandidatePose build_dynamic_bind_candidate_pose_from_world_point(
    const std::vector<tie_robot_msgs::PointCoords>& seed_world_points,
    const tf2::Transform& gripper_from_scepter,
    const DynamicBindPlannerConfig& config)
{
    DynamicBindPlanningCandidatePose candidate_pose;
    if (seed_world_points.empty()) {
        return candidate_pose;
    }

    double average_world_x = 0.0;
    double average_world_y = 0.0;
    double average_world_z = 0.0;
    for (const auto& world_point : seed_world_points) {
        average_world_x += static_cast<double>(world_point.World_coord[0]);
        average_world_y += static_cast<double>(world_point.World_coord[1]);
        average_world_z += static_cast<double>(world_point.World_coord[2]);
    }
    average_world_x /= static_cast<double>(seed_world_points.size());
    average_world_y /= static_cast<double>(seed_world_points.size());
    average_world_z /= static_cast<double>(seed_world_points.size());

    const tf2::Vector3 desired_point_in_gripper_frame(
        static_cast<double>(config.template_center_x_mm) / 1000.0,
        static_cast<double>(config.template_center_y_mm) / 1000.0,
        static_cast<double>(config.template_center_z_mm) / 1000.0);
    const tf2::Vector3 desired_point_in_scepter_frame =
        gripper_from_scepter.inverse() * desired_point_in_gripper_frame;

    candidate_pose.cabin_x = static_cast<float>(
        average_world_x - desired_point_in_scepter_frame.x() * 1000.0);
    candidate_pose.cabin_y = static_cast<float>(
        average_world_y - desired_point_in_scepter_frame.y() * 1000.0);
    candidate_pose.cabin_z = static_cast<float>(
        average_world_z +
        desired_point_in_scepter_frame.z() * 1000.0 +
        gripper_from_scepter.getOrigin().z() * 1000.0);
    return candidate_pose;
}

std::vector<PseudoSlamCandidatePoint> collect_world_points_coverable_by_dynamic_pose(
    const std::vector<tie_robot_msgs::PointCoords>& planning_world_points,
    const DynamicBindPlanningCandidatePose& candidate_pose,
    const tf2::Transform& gripper_from_scepter,
    const DynamicBindPlannerConfig& config)
{
    const CabinPoint candidate_cabin_point{candidate_pose.cabin_x, candidate_pose.cabin_y};
    return build_area_bind_candidates(
        planning_world_points,
        candidate_cabin_point,
        candidate_pose.cabin_z,
        gripper_from_scepter,
        config);
}

}  // namespace internal
}  // namespace planning
}  // namespace tie_robot_process
