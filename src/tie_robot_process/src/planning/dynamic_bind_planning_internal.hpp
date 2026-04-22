#pragma once

#include "tie_robot_process/planning/dynamic_bind_planning.hpp"

#include <tuple>
#include <unordered_set>
#include <vector>

namespace tie_robot_process {
namespace planning {
namespace internal {

struct PseudoSlamCandidatePoint
{
    tie_robot_msgs::PointCoords world_point;
    tie_robot_msgs::PointCoords local_point;
    size_t remaining_index = 0;
};

struct DynamicBindPlanningCandidatePose
{
    float cabin_x = 0.0f;
    float cabin_y = 0.0f;
    float cabin_z = 0.0f;
};

float clamp_bind_execution_cabin_z(float planned_cabin_z, const DynamicBindPlannerConfig& config);
bool is_local_bind_point_in_range(
    const tie_robot_msgs::PointCoords& point,
    const DynamicBindPlannerConfig& config);
void transform_cabin_world_point_to_planned_gripper_point(
    const tie_robot_msgs::PointCoords& world_point,
    const CabinPoint& cabin_point,
    float cabin_height,
    const tf2::Transform& gripper_from_scepter,
    tie_robot_msgs::PointCoords& gripper_point);
double local_origin_distance_sq(const tie_robot_msgs::PointCoords& point);

std::vector<std::vector<int>> group_candidate_indices_by_axis(
    const std::vector<PseudoSlamCandidatePoint>& candidates,
    int axis_index,
    float threshold);
std::vector<std::tuple<int, int, float>> match_candidate_indices_between_rows(
    const std::vector<PseudoSlamCandidatePoint>& candidates,
    const std::vector<int>& upper_row,
    const std::vector<int>& lower_row,
    float column_threshold);
std::vector<int> sort_matrix_candidate_indices(
    const std::vector<PseudoSlamCandidatePoint>& candidates,
    const std::vector<int>& matrix_indices);
std::vector<double> build_matrix_candidate_score(
    const std::vector<PseudoSlamCandidatePoint>& candidates,
    const std::vector<int>& sorted_matrix_indices,
    const std::vector<float>& column_gaps);
std::vector<int> select_nearest_origin_matrix_candidate_indices(
    const std::vector<PseudoSlamCandidatePoint>& candidates,
    const DynamicBindPlannerConfig& config);

std::vector<PseudoSlamCandidatePoint> build_area_bind_candidates(
    const std::vector<tie_robot_msgs::PointCoords>& remaining_world_points,
    const CabinPoint& cabin_point,
    float cabin_height,
    const tf2::Transform& gripper_from_scepter,
    const DynamicBindPlannerConfig& config);
std::vector<tie_robot_msgs::PointCoords> collect_dynamic_bind_seed_world_points(
    const std::vector<tie_robot_msgs::PointCoords>& planning_world_points,
    const std::unordered_set<int>& unfinished_global_indices,
    const tie_robot_msgs::PointCoords& seed_world_point,
    size_t max_seed_count);
DynamicBindPlanningCandidatePose build_dynamic_bind_candidate_pose_from_world_point(
    const std::vector<tie_robot_msgs::PointCoords>& seed_world_points,
    const tf2::Transform& gripper_from_scepter,
    const DynamicBindPlannerConfig& config);
std::vector<PseudoSlamCandidatePoint> collect_world_points_coverable_by_dynamic_pose(
    const std::vector<tie_robot_msgs::PointCoords>& planning_world_points,
    const DynamicBindPlanningCandidatePose& candidate_pose,
    const tf2::Transform& gripper_from_scepter,
    const DynamicBindPlannerConfig& config);
PseudoSlamBindGroup build_dynamic_four_point_template_group(
    const std::vector<PseudoSlamCandidatePoint>& coverable_candidates,
    const std::unordered_set<int>& unfinished_global_indices,
    int group_index,
    const DynamicBindPlannerConfig& config,
    std::unordered_set<int>& consumed_unfinished_global_indices);

}  // namespace internal
}  // namespace planning
}  // namespace tie_robot_process
