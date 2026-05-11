#include "suoqu_runtime_internal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace {

constexpr float kPrecomputedGroupSnakeRowToleranceMm = 40.0f;

float point_json_tcp_axis_value(const nlohmann::json& point_json, const char* axis_name)
{
    return point_json.value(axis_name, 0.0f);
}

int point_json_execution_order_idx(const nlohmann::json& point_json)
{
    return point_json.value(
        "local_idx",
        point_json.value("idx", point_json.value("global_idx", -1))
    );
}

}  // namespace

bool is_jump_bind_target_parity(int checkerboard_parity)
{
    return checkerboard_parity == 0;
}

std::string checkerboard_color_from_parity(int checkerboard_parity)
{
    if (checkerboard_parity == 0) {
        return "black";
    }
    if (checkerboard_parity == 1) {
        return "white";
    }
    return "unknown";
}

bool point_json_is_jump_bind_target(const nlohmann::json& point_json)
{
    if (point_json.contains("jump_bind") && point_json["jump_bind"].is_boolean()) {
        return point_json["jump_bind"].get<bool>();
    }
    if (point_json.contains("checkerboard_color") && point_json["checkerboard_color"].is_string()) {
        const std::string checkerboard_color = point_json["checkerboard_color"].get<std::string>();
        if (checkerboard_color == "black") {
            return true;
        }
        if (checkerboard_color == "white") {
            return false;
        }
    }
    return is_jump_bind_target_parity(point_json.value("checkerboard_parity", 0));
}

bool point_json_matches_jump_bind_parity(
    const nlohmann::json& point_json,
    int selected_checkerboard_parity
)
{
    const bool selected_black = selected_checkerboard_parity != 1;
    if (point_json.contains("checkerboard_parity") && point_json["checkerboard_parity"].is_number_integer()) {
        return point_json["checkerboard_parity"].get<int>() == (selected_black ? 0 : 1);
    }
    if (point_json.contains("checkerboard_color") && point_json["checkerboard_color"].is_string()) {
        const std::string checkerboard_color = point_json["checkerboard_color"].get<std::string>();
        if (checkerboard_color == "black") {
            return selected_black;
        }
        if (checkerboard_color == "white") {
            return !selected_black;
        }
    }
    const bool black_jump_bind_target = point_json_is_jump_bind_target(point_json);
    return selected_black ? black_jump_bind_target : !black_jump_bind_target;
}

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

void sort_precomputed_group_points_by_tcp_snake_rows(nlohmann::json& points_json)
{
    if (!points_json.is_array() || points_json.size() < 2U) {
        return;
    }

    std::vector<nlohmann::json> points;
    points.reserve(points_json.size());
    for (const auto& point_json : points_json) {
        points.push_back(point_json);
    }

    const auto compare_point_by_x_then_y = [](const nlohmann::json& lhs, const nlohmann::json& rhs) {
        const float lhs_x = point_json_tcp_axis_value(lhs, "x");
        const float rhs_x = point_json_tcp_axis_value(rhs, "x");
        if (std::fabs(lhs_x - rhs_x) > 1e-6f) {
            return lhs_x < rhs_x;
        }
        const float lhs_y = point_json_tcp_axis_value(lhs, "y");
        const float rhs_y = point_json_tcp_axis_value(rhs, "y");
        if (std::fabs(lhs_y - rhs_y) > 1e-6f) {
            return lhs_y < rhs_y;
        }
        return point_json_execution_order_idx(lhs) < point_json_execution_order_idx(rhs);
    };

    std::sort(points.begin(), points.end(), compare_point_by_x_then_y);

    struct SnakeRow
    {
        float mean_x = 0.0f;
        std::vector<nlohmann::json> points;
    };

    std::vector<SnakeRow> rows;
    for (const auto& point_json : points) {
        const float point_x = point_json.value("x", 0.0f);
        if (rows.empty() ||
            std::fabs(point_x - rows.back().mean_x) > kPrecomputedGroupSnakeRowToleranceMm) {
            SnakeRow row;
            row.mean_x = point_x;
            row.points.push_back(point_json);
            rows.push_back(std::move(row));
            continue;
        }

        auto& row = rows.back();
        row.points.push_back(point_json);
        row.mean_x =
            (row.mean_x * static_cast<float>(row.points.size() - 1U) + point_x) /
            static_cast<float>(row.points.size());
    }

    points_json = nlohmann::json::array();
    for (size_t row_index = 0; row_index < rows.size(); ++row_index) {
        auto& row_points = rows[row_index].points;
        const bool ascending_y = (row_index % 2U) == 0U;
        std::sort(row_points.begin(), row_points.end(), [&](const nlohmann::json& lhs, const nlohmann::json& rhs) {
            const float lhs_y = lhs.value("y", 0.0f);
            const float rhs_y = rhs.value("y", 0.0f);
            if (std::fabs(lhs_y - rhs_y) > 1e-6f) {
                return ascending_y ? lhs_y < rhs_y : lhs_y > rhs_y;
            }
            const float lhs_x = lhs.value("x", 0.0f);
            const float rhs_x = rhs.value("x", 0.0f);
            if (std::fabs(lhs_x - rhs_x) > 1e-6f) {
                return lhs_x < rhs_x;
            }
            return point_json_execution_order_idx(lhs) < point_json_execution_order_idx(rhs);
        });
        for (const auto& point_json : row_points) {
            points_json.push_back(point_json);
        }
    }
}


nlohmann::json filter_precomputed_group_points_for_execution(
    const nlohmann::json& group_json,
    const BindExecutionMemory& memory,
    const std::unordered_set<int>& blocked_global_indices,
    bool jump_bind_enabled,
    int selected_jump_bind_parity
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

        if (jump_bind_enabled && !point_json_matches_jump_bind_parity(point_json, selected_jump_bind_parity)) {
            continue;
        }

        const int global_row = point_json.value("global_row", -1);
        const int global_col = point_json.value("global_col", -1);
        const int recognition_pose_index = point_json.value("recognition_pose_index", 1);
        if (is_point_already_executed(memory, recognition_pose_index, global_row, global_col)) {
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

    sort_precomputed_group_points_by_tcp_snake_rows(filtered_points);
    return filtered_points;
}

std::unordered_set<int> collect_blocked_execution_global_indices_from_points_json(
    const nlohmann::json& points_json
)
{
    (void)points_json;
    std::unordered_set<int> blocked_global_indices;
    return blocked_global_indices;
}
