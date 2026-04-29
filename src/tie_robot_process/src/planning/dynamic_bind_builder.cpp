#include "dynamic_bind_planning_internal.hpp"

#include <algorithm>
#include <numeric>

namespace tie_robot_process {
namespace planning {
namespace internal {

std::vector<tie_robot_msgs::PointCoords> collect_dynamic_bind_seed_world_points(
    const std::vector<tie_robot_msgs::PointCoords>& planning_world_points,
    const std::unordered_set<int>& unfinished_global_indices,
    const tie_robot_msgs::PointCoords& seed_world_point,
    size_t max_seed_count)
{
    std::vector<std::pair<double, tie_robot_msgs::PointCoords>> sorted_seed_candidates;
    for (const auto& world_point : planning_world_points) {
        if (unfinished_global_indices.count(world_point.idx) <= 0) {
            continue;
        }
        const double dx = static_cast<double>(world_point.World_coord[0] - seed_world_point.World_coord[0]);
        const double dy = static_cast<double>(world_point.World_coord[1] - seed_world_point.World_coord[1]);
        const double dz = static_cast<double>(world_point.World_coord[2] - seed_world_point.World_coord[2]);
        sorted_seed_candidates.push_back({dx * dx + dy * dy + dz * dz, world_point});
    }

    std::sort(sorted_seed_candidates.begin(), sorted_seed_candidates.end(), [](const auto& lhs, const auto& rhs) {
        if (lhs.first != rhs.first) {
            return lhs.first < rhs.first;
        }
        return lhs.second.idx < rhs.second.idx;
    });

    std::vector<tie_robot_msgs::PointCoords> seed_world_points;
    const size_t seed_count = std::min(sorted_seed_candidates.size(), max_seed_count);
    seed_world_points.reserve(seed_count);
    for (size_t seed_index = 0; seed_index < seed_count; ++seed_index) {
        seed_world_points.push_back(sorted_seed_candidates[seed_index].second);
    }
    return seed_world_points;
}

PseudoSlamBindGroup build_dynamic_four_point_template_group(
    const std::vector<PseudoSlamCandidatePoint>& coverable_candidates,
    const std::unordered_set<int>& unfinished_global_indices,
    int group_index,
    const DynamicBindPlannerConfig& config,
    std::unordered_set<int>& consumed_unfinished_global_indices)
{
    PseudoSlamBindGroup bind_group;
    bind_group.group_index = group_index;
    consumed_unfinished_global_indices.clear();

    std::vector<PseudoSlamCandidatePoint> unfinished_candidates;
    for (const auto& candidate : coverable_candidates) {
        if (unfinished_global_indices.count(candidate.world_point.idx) > 0) {
            unfinished_candidates.push_back(candidate);
        }
    }

    std::vector<int> selected_unfinished_candidate_indices =
        select_nearest_origin_matrix_candidate_indices(unfinished_candidates, config);
    if (selected_unfinished_candidate_indices.size() != 4) {
        return bind_group;
    }

    bind_group.group_type = "matrix_2x2";
    for (const int candidate_index : selected_unfinished_candidate_indices) {
        const auto& world_point =
            unfinished_candidates[static_cast<size_t>(candidate_index)].world_point;
        bind_group.bind_points_world.push_back(world_point);
        consumed_unfinished_global_indices.insert(world_point.idx);
    }
    return bind_group;
}

}  // namespace internal
}  // namespace planning
}  // namespace tie_robot_process
