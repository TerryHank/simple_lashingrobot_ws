#include "suoqu_runtime_internal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

float clamp_bind_execution_cabin_z(float planned_cabin_z)
{
    return std::max(planned_cabin_z, kBindExecutionCabinMinZMm);
}

bool assign_planned_gripper_coords_to_bind_point_json(
    nlohmann::json& point_json,
    const Cabin_Point& cabin_point,
    float cabin_z,
    const tf2::Transform& gripper_from_base_link
)
{
    tie_robot_msgs::PointCoords world_point;
    world_point.idx = point_json.value(
        "global_idx",
        point_json.value("local_idx", point_json.value("idx", 1))
    );
    world_point.Pix_coord[0] = 0;
    world_point.Pix_coord[1] = 0;
    world_point.World_coord[0] = point_json.value("world_x", point_json.value("x", 0.0f));
    world_point.World_coord[1] = point_json.value("world_y", point_json.value("y", 0.0f));
    world_point.World_coord[2] = point_json.value("world_z", point_json.value("z", 0.0f));
    world_point.Angle = point_json.value("angle", -45.0f);
    world_point.is_shuiguan = false;

    tie_robot_msgs::PointCoords gripper_point;
    if (!transform_cabin_world_point_to_planned_gripper_point(
            world_point,
            cabin_point,
            clamp_bind_execution_cabin_z(cabin_z),
            gripper_from_base_link,
            gripper_point
        )) {
        return false;
    }

    point_json["x"] = gripper_point.World_coord[0];
    point_json["y"] = gripper_point.World_coord[1];
    point_json["z"] = gripper_point.World_coord[2];
    return true;
}

bool assign_planned_gripper_coords_to_bind_point_json(
    nlohmann::json& point_json,
    const tie_robot_process::planning::CabinPoint& cabin_point,
    float cabin_z,
    const tf2::Transform& gripper_from_base_link
)
{
    const Cabin_Point legacy_cabin_point{cabin_point.x, cabin_point.y};
    return assign_planned_gripper_coords_to_bind_point_json(
        point_json,
        legacy_cabin_point,
        cabin_z,
        gripper_from_base_link
    );
}

void align_execution_path_origin_xy_to_first_area_if_needed(
    const nlohmann::json& areas_json,
    float& path_origin_x,
    float& path_origin_y,
    const char* execution_mode_name
)
{
    if (!areas_json.is_array() || areas_json.empty()) {
        return;
    }
    if (!areas_json.front().contains("cabin_pose") || !areas_json.front()["cabin_pose"].is_object()) {
        return;
    }

    const auto& first_cabin_pose = areas_json.front()["cabin_pose"];
    const float first_area_x = first_cabin_pose.value("x", path_origin_x);
    const float first_area_y = first_cabin_pose.value("y", path_origin_y);
    if (std::fabs(path_origin_x - first_area_x) < 1e-3f &&
        std::fabs(path_origin_y - first_area_y) < 1e-3f) {
        return;
    }

    printCurrentTime();
    ros_log_printf(
        "Cabin_Warn: %s检测到path_origin.xy=(%f,%f)与首个执行区域xy=(%f,%f)不一致，按首个真实执行区域修正起点XY。\n",
        execution_mode_name,
        path_origin_x,
        path_origin_y,
        first_area_x,
        first_area_y
    );
    path_origin_x = first_area_x;
    path_origin_y = first_area_y;
}


nlohmann::json filter_precomputed_group_points_for_execution(
    const nlohmann::json& group_json,
    const BindExecutionMemory& memory,
    const std::unordered_set<int>& blocked_global_indices,
    bool only_checkerboard_parity_zero
)
{
    nlohmann::json filtered_points = nlohmann::json::array();
    if (!group_json.contains("points") || !group_json["points"].is_array()) {
        return filtered_points;
    }

    std::unordered_set<long long> current_batch_checkerboard_cells;
    for (const auto& point_json : group_json["points"]) {
        const int global_idx = point_json.value("global_idx", point_json.value("idx", -1));
        if (global_idx > 0 && blocked_global_indices.count(global_idx) > 0) {
            continue;
        }

        if (only_checkerboard_parity_zero && point_json.value("checkerboard_parity", 0) != 0) {
            continue;
        }

        const int global_row = point_json.value("global_row", -1);
        const int global_col = point_json.value("global_col", -1);
        if (is_point_already_executed(memory, global_row, global_col)) {
            continue;
        }
        if (global_row >= 0 && global_col >= 0) {
            const long long checkerboard_cell_key =
                encode_checkerboard_cell_key(global_row, global_col);
            const bool inserted = current_batch_checkerboard_cells.insert(checkerboard_cell_key).second;
            if (!inserted) {
                continue;
            }
        }

        filtered_points.push_back(point_json);
    }

    return filtered_points;
}

std::unordered_set<int> collect_blocked_execution_global_indices_from_points_json(
    const nlohmann::json& points_json
)
{
    std::unordered_set<int> blocked_global_indices;
    if (!points_json.contains("pseudo_slam_points") || !points_json["pseudo_slam_points"].is_array()) {
        return blocked_global_indices;
    }

    for (const auto& point_json : points_json["pseudo_slam_points"]) {
        const int global_idx = point_json.value("global_idx", point_json.value("idx", -1));
        if (global_idx <= 0) {
            continue;
        }

        const bool is_blocked =
            point_json.value("is_planning_outlier", false) ||
            point_json.value("is_planning_outlier_line_member", false) ||
            point_json.value("is_outlier_secondary_plane_member", false) ||
            point_json.value("is_outlier_column_neighbor_blocked", false) ||
            !point_json.value("is_planning_checkerboard_member", false);
        if (is_blocked) {
            blocked_global_indices.insert(global_idx);
        }
    }

    return blocked_global_indices;
}
