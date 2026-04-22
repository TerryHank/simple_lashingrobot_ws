#include "dynamic_bind_planning_internal.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace tie_robot_process {
namespace planning {

std::vector<PseudoSlamGroupedAreaEntry> build_dynamic_bind_area_entries_from_scan_world(
    const std::vector<tie_robot_msgs::PointCoords>& planning_world_points,
    const CabinPoint& path_origin,
    float cabin_height,
    const tf2::Transform& gripper_from_scepter,
    const DynamicBindPlannerConfig& config)
{
    using namespace internal;

    std::vector<PseudoSlamGroupedAreaEntry> bind_area_entries;
    if (planning_world_points.empty()) {
        return bind_area_entries;
    }

    std::unordered_set<int> unfinished_global_indices;
    for (const auto& world_point : planning_world_points) {
        unfinished_global_indices.insert(world_point.idx);
    }

    int area_index = 1;
    while (!unfinished_global_indices.empty()) {
        const auto seed_point_it = std::min_element(
            planning_world_points.begin(),
            planning_world_points.end(),
            [&](const auto& lhs, const auto& rhs) {
                const bool lhs_unfinished = unfinished_global_indices.count(lhs.idx) > 0;
                const bool rhs_unfinished = unfinished_global_indices.count(rhs.idx) > 0;
                if (lhs_unfinished != rhs_unfinished) {
                    return lhs_unfinished;
                }
                const double lhs_distance_sq =
                    static_cast<double>(lhs.World_coord[0] - path_origin.x) *
                        static_cast<double>(lhs.World_coord[0] - path_origin.x) +
                    static_cast<double>(lhs.World_coord[1] - path_origin.y) *
                        static_cast<double>(lhs.World_coord[1] - path_origin.y);
                const double rhs_distance_sq =
                    static_cast<double>(rhs.World_coord[0] - path_origin.x) *
                        static_cast<double>(rhs.World_coord[0] - path_origin.x) +
                    static_cast<double>(rhs.World_coord[1] - path_origin.y) *
                        static_cast<double>(rhs.World_coord[1] - path_origin.y);
                if (lhs_distance_sq != rhs_distance_sq) {
                    return lhs_distance_sq < rhs_distance_sq;
                }
                return lhs.idx < rhs.idx;
            });
        if (seed_point_it == planning_world_points.end() ||
            unfinished_global_indices.count(seed_point_it->idx) <= 0) {
            break;
        }

        const std::vector<tie_robot_msgs::PointCoords> seed_world_points =
            collect_dynamic_bind_seed_world_points(
                planning_world_points,
                unfinished_global_indices,
                *seed_point_it,
                static_cast<size_t>(std::max(config.seed_neighbor_count, 1)));
        if (seed_world_points.empty()) {
            unfinished_global_indices.erase(seed_point_it->idx);
            continue;
        }

        DynamicBindPlanningCandidatePose best_candidate_pose;
        std::vector<PseudoSlamCandidatePoint> best_coverable_candidates;
        size_t best_unfinished_coverable_count = 0;
        for (size_t seed_count = 1; seed_count <= seed_world_points.size(); ++seed_count) {
            const std::vector<tie_robot_msgs::PointCoords> current_seed_world_points(
                seed_world_points.begin(),
                seed_world_points.begin() + static_cast<long>(seed_count));
            const DynamicBindPlanningCandidatePose candidate_pose =
                build_dynamic_bind_candidate_pose_from_world_point(
                    current_seed_world_points,
                    gripper_from_scepter,
                    config);
            const std::vector<PseudoSlamCandidatePoint> coverable_candidates =
                collect_world_points_coverable_by_dynamic_pose(
                    planning_world_points,
                    candidate_pose,
                    gripper_from_scepter,
                    config);
            size_t unfinished_coverable_count = 0;
            for (const auto& candidate : coverable_candidates) {
                if (unfinished_global_indices.count(candidate.world_point.idx) > 0) {
                    unfinished_coverable_count++;
                }
            }
            if (unfinished_coverable_count > best_unfinished_coverable_count ||
                (unfinished_coverable_count == best_unfinished_coverable_count &&
                 coverable_candidates.size() > best_coverable_candidates.size())) {
                best_candidate_pose = candidate_pose;
                best_coverable_candidates = coverable_candidates;
                best_unfinished_coverable_count = unfinished_coverable_count;
            }
        }

        if (best_coverable_candidates.empty()) {
            unfinished_global_indices.erase(seed_point_it->idx);
            continue;
        }

        std::unordered_set<int> consumed_unfinished_global_indices;
        PseudoSlamBindGroup bind_group = build_dynamic_four_point_template_group(
            best_coverable_candidates,
            unfinished_global_indices,
            1,
            config,
            consumed_unfinished_global_indices);
        if (bind_group.bind_points_world.empty()) {
            unfinished_global_indices.erase(seed_point_it->idx);
            continue;
        }

        for (const int consumed_global_idx : consumed_unfinished_global_indices) {
            unfinished_global_indices.erase(consumed_global_idx);
        }

        PseudoSlamGroupedAreaEntry area_entry;
        area_entry.area_index = area_index++;
        area_entry.cabin_point = {best_candidate_pose.cabin_x, best_candidate_pose.cabin_y};
        area_entry.cabin_z = clamp_bind_execution_cabin_z(
            best_candidate_pose.cabin_z > 0.0f ? best_candidate_pose.cabin_z : cabin_height,
            config);
        area_entry.bind_groups.push_back(bind_group);
        bind_area_entries.push_back(area_entry);
    }

    return bind_area_entries;
}

BindExecutionPathOriginPose build_dynamic_bind_execution_path_origin(
    const std::vector<PseudoSlamGroupedAreaEntry>& bind_area_entries,
    const CabinPoint& planning_reference_origin,
    float cabin_height,
    float bind_execution_cabin_min_z_mm)
{
    BindExecutionPathOriginPose execution_path_origin;
    execution_path_origin.x = planning_reference_origin.x;
    execution_path_origin.y = planning_reference_origin.y;
    execution_path_origin.z = std::max(cabin_height, bind_execution_cabin_min_z_mm);

    if (bind_area_entries.empty()) {
        return execution_path_origin;
    }

    execution_path_origin.x = bind_area_entries.front().cabin_point.x;
    execution_path_origin.y = bind_area_entries.front().cabin_point.y;
    return execution_path_origin;
}

void sort_bind_area_entries_by_snake_rows(
    std::vector<PseudoSlamGroupedAreaEntry>& bind_area_entries,
    float row_tolerance_mm)
{
    if (bind_area_entries.empty()) {
        return;
    }

    std::sort(bind_area_entries.begin(), bind_area_entries.end(), [&](const PseudoSlamGroupedAreaEntry& lhs, const PseudoSlamGroupedAreaEntry& rhs) {
        if (lhs.cabin_point.y != rhs.cabin_point.y) {
            return lhs.cabin_point.y < rhs.cabin_point.y;
        }
        if (lhs.cabin_point.x != rhs.cabin_point.x) {
            return lhs.cabin_point.x < rhs.cabin_point.x;
        }
        return lhs.area_index < rhs.area_index;
    });

    std::vector<std::vector<PseudoSlamGroupedAreaEntry>> snake_rows;
    std::vector<float> row_mean_y_values;
    for (const auto& area_entry : bind_area_entries) {
        if (snake_rows.empty() ||
            std::fabs(area_entry.cabin_point.y - row_mean_y_values.back()) > row_tolerance_mm) {
            snake_rows.push_back({area_entry});
            row_mean_y_values.push_back(area_entry.cabin_point.y);
            continue;
        }

        auto& row_entries = snake_rows.back();
        row_entries.push_back(area_entry);
        const float updated_row_mean_y =
            (row_mean_y_values.back() * static_cast<float>(row_entries.size() - 1) +
             area_entry.cabin_point.y) /
            static_cast<float>(row_entries.size());
        row_mean_y_values.back() = updated_row_mean_y;
    }

    bind_area_entries.clear();
    int reordered_area_index = 1;
    for (size_t row_index = 0; row_index < snake_rows.size(); ++row_index) {
        auto& row_entries = snake_rows[row_index];
        std::sort(row_entries.begin(), row_entries.end(), [&](const PseudoSlamGroupedAreaEntry& lhs, const PseudoSlamGroupedAreaEntry& rhs) {
            if ((row_index % 2U) == 0U) {
                if (lhs.cabin_point.x != rhs.cabin_point.x) {
                    return lhs.cabin_point.x < rhs.cabin_point.x;
                }
            } else {
                if (lhs.cabin_point.x != rhs.cabin_point.x) {
                    return lhs.cabin_point.x > rhs.cabin_point.x;
                }
            }
            if (lhs.cabin_point.y != rhs.cabin_point.y) {
                return lhs.cabin_point.y < rhs.cabin_point.y;
            }
            return lhs.area_index < rhs.area_index;
        });

        for (auto& area_entry : row_entries) {
            area_entry.area_index = reordered_area_index++;
            bind_area_entries.push_back(area_entry);
        }
    }
}

}  // namespace planning
}  // namespace tie_robot_process
