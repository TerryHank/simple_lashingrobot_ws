#include "suoqu_runtime_internal.hpp"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace
{
constexpr int kPseudoSlamPoseGlobalIndexStride = 10000;

int normalize_artifact_pose_index(int recognition_pose_index)
{
    return recognition_pose_index > 0 ? recognition_pose_index : 1;
}

int pose_global_index_offset(int recognition_pose_index)
{
    return (normalize_artifact_pose_index(recognition_pose_index) - 1) *
           kPseudoSlamPoseGlobalIndexStride;
}

int read_positive_json_int(const nlohmann::json& value, const char* key, int fallback)
{
    const int raw_value = value.value(key, fallback);
    return raw_value > 0 ? raw_value : fallback;
}

int read_nonnegative_json_int(const nlohmann::json& value, const char* key, int fallback)
{
    const int raw_value = value.value(key, fallback);
    return raw_value >= 0 ? raw_value : fallback;
}

int infer_scan_pose_group_index(const nlohmann::json& scan_pose_group)
{
    return normalize_artifact_pose_index(
        scan_pose_group.value(
            "pose_index",
            scan_pose_group.value("recognition_pose_index", 1)
        )
    );
}

void ensure_pose_local_point_identity(nlohmann::json& point_json, int pose_index, int fallback_local_idx)
{
    const int local_idx = read_positive_json_int(
        point_json,
        "pose_local_global_idx",
        read_positive_json_int(
            point_json,
            "global_idx",
            read_positive_json_int(point_json, "idx", fallback_local_idx)
        )
    );
    const int local_row = read_nonnegative_json_int(
        point_json,
        "pose_local_global_row",
        read_nonnegative_json_int(point_json, "global_row", -1)
    );
    const int local_col = read_nonnegative_json_int(
        point_json,
        "pose_local_global_col",
        read_nonnegative_json_int(point_json, "global_col", -1)
    );
    const int global_offset = pose_global_index_offset(pose_index);

    point_json["pose_index"] = pose_index;
    point_json["recognition_pose_index"] = pose_index;
    point_json["pose_local_global_idx"] = local_idx;
    point_json["global_idx"] = global_offset + local_idx;
    if (local_row >= 0) {
        point_json["pose_local_global_row"] = local_row;
        point_json["global_row"] = global_offset + local_row;
    }
    if (local_col >= 0) {
        point_json["pose_local_global_col"] = local_col;
        point_json["global_col"] = global_offset + local_col;
    }
}

bool load_existing_scan_pose_groups(
    const std::string& artifact_path,
    const std::string& legacy_payload_key,
    std::vector<nlohmann::json>& scan_pose_groups,
    std::string* error_message
)
{
    scan_pose_groups.clear();

    std::ifstream artifact_file(artifact_path);
    if (!artifact_file.is_open()) {
        return true;
    }

    try {
        nlohmann::json artifact_json;
        artifact_file >> artifact_json;
        if (artifact_file.fail() && !artifact_file.eof()) {
            throw std::runtime_error("artifact read failure");
        }
        if (!artifact_json.is_object()) {
            throw std::runtime_error("artifact root must be object");
        }

        if (artifact_json.contains("scan_pose_groups") &&
            artifact_json["scan_pose_groups"].is_array()) {
            for (auto scan_pose_group : artifact_json["scan_pose_groups"]) {
                if (scan_pose_group.is_object()) {
                    scan_pose_groups.push_back(std::move(scan_pose_group));
                }
            }
            return true;
        }

        if (artifact_json.contains(legacy_payload_key) &&
            artifact_json[legacy_payload_key].is_array()) {
            nlohmann::json legacy_group;
            legacy_group["pose_index"] = 1;
            legacy_group["recognition_pose_index"] = 1;
            legacy_group["scan_session_id"] = artifact_json.value("scan_session_id", std::string());
            legacy_group["path_signature"] = artifact_json.value("path_signature", std::string());
            if (artifact_json.contains("path_origin")) {
                legacy_group["path_origin"] = artifact_json["path_origin"];
            }
            if (artifact_json.contains("cabin_height")) {
                legacy_group["cabin_height"] = artifact_json["cabin_height"];
            }
            if (artifact_json.contains("cabin_speed")) {
                legacy_group["cabin_speed"] = artifact_json["cabin_speed"];
            }
            legacy_group[legacy_payload_key] = artifact_json[legacy_payload_key];
            scan_pose_groups.push_back(std::move(legacy_group));
        }
    } catch (const std::exception&) {
        if (error_message != nullptr) {
            *error_message = "读取现有扫描账本失败，已拒绝覆盖以避免丢失其他位姿数据";
        }
        scan_pose_groups.clear();
        return false;
    }

    return true;
}

void replace_scan_pose_group_by_index(
    std::vector<nlohmann::json>& scan_pose_groups,
    int recognition_pose_index,
    const nlohmann::json& current_scan_pose_group
)
{
    const int normalized_pose_index = normalize_artifact_pose_index(recognition_pose_index);
    scan_pose_groups.erase(
        std::remove_if(
            scan_pose_groups.begin(),
            scan_pose_groups.end(),
            [normalized_pose_index](const nlohmann::json& scan_pose_group) {
                return infer_scan_pose_group_index(scan_pose_group) == normalized_pose_index;
            }
        ),
        scan_pose_groups.end()
    );

    scan_pose_groups.push_back(current_scan_pose_group);
    std::sort(
        scan_pose_groups.begin(),
        scan_pose_groups.end(),
        [](const nlohmann::json& lhs, const nlohmann::json& rhs) {
            return infer_scan_pose_group_index(lhs) < infer_scan_pose_group_index(rhs);
        }
    );
}

nlohmann::json flatten_scan_pose_groups_points(std::vector<nlohmann::json>& scan_pose_groups)
{
    nlohmann::json flattened_points = nlohmann::json::array();
    for (auto& scan_pose_group : scan_pose_groups) {
        const int pose_index = infer_scan_pose_group_index(scan_pose_group);
        scan_pose_group["pose_index"] = pose_index;
        scan_pose_group["recognition_pose_index"] = pose_index;
        if (!scan_pose_group.contains("pseudo_slam_points") ||
            !scan_pose_group["pseudo_slam_points"].is_array()) {
            scan_pose_group["pseudo_slam_points"] = nlohmann::json::array();
        }

        int fallback_local_idx = 1;
        for (auto& point_json : scan_pose_group["pseudo_slam_points"]) {
            if (!point_json.is_object()) {
                fallback_local_idx++;
                continue;
            }
            ensure_pose_local_point_identity(point_json, pose_index, fallback_local_idx);
            point_json["idx"] = point_json.value("global_idx", fallback_local_idx);
            flattened_points.push_back(point_json);
            fallback_local_idx++;
        }
    }
    return flattened_points;
}

nlohmann::json build_scan_pose_groups_by_pose_index(
    const std::vector<nlohmann::json>& scan_pose_groups
)
{
    nlohmann::json groups_by_pose_index = nlohmann::json::object();
    for (const auto& scan_pose_group : scan_pose_groups) {
        const int pose_index = infer_scan_pose_group_index(scan_pose_group);
        groups_by_pose_index[std::to_string(pose_index)] = scan_pose_group;
    }
    return groups_by_pose_index;
}

nlohmann::json flatten_scan_pose_groups_bind_path(
    std::vector<nlohmann::json>& scan_pose_groups,
    BindExecutionPathOriginPose* merged_path_origin_out
)
{
    nlohmann::json flattened_areas = nlohmann::json::array();
    bool path_origin_selected = false;
    int next_area_index = 1;

    for (auto& scan_pose_group : scan_pose_groups) {
        const int pose_index = infer_scan_pose_group_index(scan_pose_group);
        scan_pose_group["pose_index"] = pose_index;
        scan_pose_group["recognition_pose_index"] = pose_index;
        if (!scan_pose_group.contains("areas") ||
            !scan_pose_group["areas"].is_array()) {
            scan_pose_group["areas"] = nlohmann::json::array();
        }

        if (!path_origin_selected && merged_path_origin_out != nullptr) {
            if (scan_pose_group.contains("path_origin") &&
                scan_pose_group["path_origin"].is_object()) {
                const auto& path_origin = scan_pose_group["path_origin"];
                merged_path_origin_out->x = path_origin.value("x", 0.0f);
                merged_path_origin_out->y = path_origin.value("y", 0.0f);
                merged_path_origin_out->z = path_origin.value("z", 0.0f);
                path_origin_selected = true;
            }
        }

        for (auto& area_json : scan_pose_group["areas"]) {
            if (!area_json.is_object()) {
                continue;
            }
            const int local_area_index = area_json.value("pose_local_area_index", area_json.value("area_index", next_area_index));
            area_json["pose_index"] = pose_index;
            area_json["recognition_pose_index"] = pose_index;
            area_json["pose_local_area_index"] = local_area_index;
            area_json["area_index"] = next_area_index++;
            if (!path_origin_selected && merged_path_origin_out != nullptr &&
                area_json.contains("cabin_pose") &&
                area_json["cabin_pose"].is_object()) {
                const auto& cabin_pose = area_json["cabin_pose"];
                merged_path_origin_out->x = cabin_pose.value("x", 0.0f);
                merged_path_origin_out->y = cabin_pose.value("y", 0.0f);
                merged_path_origin_out->z = cabin_pose.value("z", 0.0f);
                path_origin_selected = true;
            }

            if (area_json.contains("groups") && area_json["groups"].is_array()) {
                int fallback_local_idx = 1;
                for (auto& group_json : area_json["groups"]) {
                    if (!group_json.is_object() ||
                        !group_json.contains("points") ||
                        !group_json["points"].is_array()) {
                        continue;
                    }
                    group_json["pose_index"] = pose_index;
                    group_json["recognition_pose_index"] = pose_index;
                    for (auto& point_json : group_json["points"]) {
                        if (!point_json.is_object()) {
                            fallback_local_idx++;
                            continue;
                        }
                        ensure_pose_local_point_identity(point_json, pose_index, fallback_local_idx);
                        fallback_local_idx++;
                    }
                }
            }

            flattened_areas.push_back(area_json);
        }
    }

    return flattened_areas;
}
}  // namespace

bool write_json_file_atomically(
    const std::string& final_path,
    const nlohmann::json& json_value,
    std::string* error_message
)
{
    if (error_message != nullptr) {
        error_message->clear();
    }

    const std::string temp_path = final_path + ".tmp";
    std::ofstream file_obj(temp_path);
    if (!file_obj.is_open()) {
        if (error_message != nullptr) {
            *error_message = "无法打开临时文件进行写入";
        }
        return false;
    }

    file_obj << json_value.dump(4);
    file_obj.flush();
    file_obj.close();
    if (!file_obj.good()) {
        std::remove(temp_path.c_str());
        if (error_message != nullptr) {
            *error_message = "写入临时文件失败";
        }
        return false;
    }
    if (std::rename(temp_path.c_str(), final_path.c_str()) != 0) {
        std::remove(temp_path.c_str());
        if (error_message != nullptr) {
            *error_message = "原子替换目标文件失败";
        }
        return false;
    }

    return true;
}

bool write_pseudo_slam_points_json(
    const std::vector<tie_robot_msgs::PointCoords>& merged_points,
    const std::unordered_map<int, PseudoSlamCheckerboardInfo>& checkerboard_info_by_idx,
    const std::unordered_map<int, PseudoSlamCheckerboardInfo>& planning_checkerboard_info_by_idx,
    const std::unordered_set<int>& outlier_secondary_plane_global_indices,
    const std::unordered_set<int>& outlier_line_global_indices,
    const std::unordered_set<int>& outlier_column_neighbor_blocked_global_indices,
    const std::string& scan_session_id,
    const std::string& path_signature,
    int recognition_pose_index,
    std::string* error_message
)
{
    (void)planning_checkerboard_info_by_idx;
    (void)outlier_secondary_plane_global_indices;
    (void)outlier_line_global_indices;
    (void)outlier_column_neighbor_blocked_global_indices;
    if (error_message != nullptr) {
        error_message->clear();
    }

    const int normalized_pose_index = normalize_artifact_pose_index(recognition_pose_index);
    nlohmann::json current_scan_pose_group;
    current_scan_pose_group["pose_index"] = normalized_pose_index;
    current_scan_pose_group["recognition_pose_index"] = normalized_pose_index;
    current_scan_pose_group["scan_session_id"] = scan_session_id;
    current_scan_pose_group["path_signature"] = path_signature;
    current_scan_pose_group["pseudo_slam_points"] = nlohmann::json::array();
    for (const auto& point : merged_points) {
        auto checkerboard_it = checkerboard_info_by_idx.find(point.idx);
        const int global_row = checkerboard_it != checkerboard_info_by_idx.end() ? checkerboard_it->second.global_row : -1;
        const int global_col = checkerboard_it != checkerboard_info_by_idx.end() ? checkerboard_it->second.global_col : -1;
        const int checkerboard_parity = checkerboard_it != checkerboard_info_by_idx.end() ? checkerboard_it->second.checkerboard_parity : -1;
        const bool jump_bind = is_jump_bind_target_parity(checkerboard_parity);
        const std::string checkerboard_color = checkerboard_color_from_parity(checkerboard_parity);
        const bool is_checkerboard_member =
            checkerboard_it != checkerboard_info_by_idx.end() ? checkerboard_it->second.is_checkerboard_member : false;
        const int local_global_idx = point.idx > 0 ? point.idx : static_cast<int>(current_scan_pose_group["pseudo_slam_points"].size()) + 1;
        const int local_global_row = global_row;
        const int local_global_col = global_col;
        nlohmann::json point_json =
            {
                {"idx", local_global_idx},
                {"global_idx", local_global_idx},
                {"pose_local_global_idx", local_global_idx},
                {"global_row", local_global_row},
                {"global_col", local_global_col},
                {"pose_local_global_row", local_global_row},
                {"pose_local_global_col", local_global_col},
                {"pose_index", normalized_pose_index},
                {"recognition_pose_index", normalized_pose_index},
                {"checkerboard_parity", checkerboard_parity},
                {"jump_bind", jump_bind},
                {"checkerboard_color", checkerboard_color},
                {"is_checkerboard_member", is_checkerboard_member},
                {"x", point.World_coord[0]},
                {"y", point.World_coord[1]},
                {"z", point.World_coord[2]},
                {"world_x", point.World_coord[0]},
                {"world_y", point.World_coord[1]},
                {"world_z", point.World_coord[2]},
                {"angle", point.Angle},
            };
        ensure_pose_local_point_identity(
            point_json,
            normalized_pose_index,
            static_cast<int>(current_scan_pose_group["pseudo_slam_points"].size()) + 1
        );
        current_scan_pose_group["pseudo_slam_points"].push_back(point_json);
    }

    std::vector<nlohmann::json> scan_pose_groups;
    if (!load_existing_scan_pose_groups(
            pseudo_slam_points_json_file,
            "pseudo_slam_points",
            scan_pose_groups,
            error_message)) {
        return false;
    }
    replace_scan_pose_group_by_index(
        scan_pose_groups,
        normalized_pose_index,
        current_scan_pose_group
    );

    nlohmann::json points_json;
    points_json["scan_session_id"] = scan_session_id;
    points_json["path_signature"] = path_signature;
    nlohmann::json flattened_points =
        flatten_scan_pose_groups_points(scan_pose_groups);
    points_json["scan_pose_groups"] = scan_pose_groups;
    points_json["scan_pose_groups_by_pose_index"] =
        build_scan_pose_groups_by_pose_index(scan_pose_groups);
    points_json["pseudo_slam_points"] = flattened_points;
    points_json["scan_pose_group_count"] = scan_pose_groups.size();

    return write_json_file_atomically(pseudo_slam_points_json_file, points_json, error_message);
}

bool write_pseudo_slam_bind_path_json(
    const std::vector<PseudoSlamGroupedAreaEntry>& area_entries,
    const std::unordered_map<int, PseudoSlamCheckerboardInfo>& checkerboard_info_by_idx,
    const BindExecutionPathOriginPose& path_origin,
    float cabin_height,
    float cabin_speed,
    const std::string& scan_session_id,
    const std::string& path_signature,
    int recognition_pose_index,
    BindExecutionPathOriginPose* merged_path_origin_out,
    std::string* error_message
)
{
    tf2::Transform gripper_from_base_link;
    if (!lookup_gripper_from_base_link_transform(gripper_from_base_link)) {
        if (error_message != nullptr) {
            *error_message = "无法获取base_link->gripper_frame变换，无法为pseudo_slam_bind_path.json写入TCP局部坐标";
        }
        return false;
    }

    const int normalized_pose_index = normalize_artifact_pose_index(recognition_pose_index);
    nlohmann::json current_scan_pose_group;
    current_scan_pose_group["pose_index"] = normalized_pose_index;
    current_scan_pose_group["recognition_pose_index"] = normalized_pose_index;
    current_scan_pose_group["scan_session_id"] = scan_session_id;
    current_scan_pose_group["path_signature"] = path_signature;
    current_scan_pose_group["scan_mode"] = "scan_only";
    current_scan_pose_group["cabin_height"] = cabin_height;
    current_scan_pose_group["cabin_speed"] = cabin_speed;
    current_scan_pose_group["path_origin"] = {
        {"x", path_origin.x},
        {"y", path_origin.y},
        {"z", path_origin.z},
    };
    current_scan_pose_group["areas"] = nlohmann::json::array();

    for (const auto& area_entry : area_entries) {
        nlohmann::json area_json;
        area_json["area_index"] = area_entry.area_index;
        area_json["pose_index"] = normalized_pose_index;
        area_json["recognition_pose_index"] = normalized_pose_index;
        area_json["pose_local_area_index"] = area_entry.area_index;
        area_json["cabin_pose"] = {
            {"x", area_entry.cabin_point.x},
            {"y", area_entry.cabin_point.y},
            {"z", area_entry.cabin_z},
        };
        area_json["groups"] = nlohmann::json::array();
        for (const auto& bind_group : area_entry.bind_groups) {
            nlohmann::json group_json;
            group_json["group_index"] = bind_group.group_index;
            group_json["group_type"] = bind_group.group_type;
            group_json["pose_index"] = normalized_pose_index;
            group_json["recognition_pose_index"] = normalized_pose_index;
            group_json["points"] = nlohmann::json::array();
            int local_idx = 1;
            for (const auto& point : bind_group.bind_points_world) {
                auto checkerboard_it = checkerboard_info_by_idx.find(point.idx);
                const int global_row = checkerboard_it != checkerboard_info_by_idx.end() ? checkerboard_it->second.global_row : -1;
                const int global_col = checkerboard_it != checkerboard_info_by_idx.end() ? checkerboard_it->second.global_col : -1;
                const int checkerboard_parity = checkerboard_it != checkerboard_info_by_idx.end() ? checkerboard_it->second.checkerboard_parity : -1;
                const bool jump_bind = is_jump_bind_target_parity(checkerboard_parity);
                const std::string checkerboard_color = checkerboard_color_from_parity(checkerboard_parity);
                const bool is_checkerboard_member =
                    checkerboard_it != checkerboard_info_by_idx.end() ? checkerboard_it->second.is_checkerboard_member : false;
                nlohmann::json point_json = {
                    {"idx", local_idx},
                    {"local_idx", local_idx},
                    {"global_idx", point.idx},
                    {"pose_local_global_idx", point.idx},
                    {"global_row", global_row},
                    {"global_col", global_col},
                    {"pose_local_global_row", global_row},
                    {"pose_local_global_col", global_col},
                    {"pose_index", normalized_pose_index},
                    {"recognition_pose_index", normalized_pose_index},
                    {"checkerboard_parity", checkerboard_parity},
                    {"jump_bind", jump_bind},
                    {"checkerboard_color", checkerboard_color},
                    {"is_checkerboard_member", is_checkerboard_member},
                    {"world_x", point.World_coord[0]},
                    {"world_y", point.World_coord[1]},
                    {"world_z", point.World_coord[2]},
                    {"angle", point.Angle},
                };
                if (!assign_planned_gripper_coords_to_bind_point_json(
                        point_json,
                        area_entry.cabin_point,
                        area_entry.cabin_z,
                        gripper_from_base_link
                    )) {
                    if (error_message != nullptr) {
                        *error_message =
                            "无法根据规划区域cabin_pose为pseudo_slam_bind_path.json生成TCP局部点坐标";
                    }
                    return false;
                }
                ensure_pose_local_point_identity(point_json, normalized_pose_index, local_idx);
                group_json["points"].push_back(point_json);
                local_idx++;
            }
            area_json["groups"].push_back(group_json);
        }
        current_scan_pose_group["areas"].push_back(area_json);
    }

    std::vector<nlohmann::json> scan_pose_groups;
    if (!load_existing_scan_pose_groups(
            pseudo_slam_bind_path_json_file,
            "areas",
            scan_pose_groups,
            error_message)) {
        return false;
    }
    replace_scan_pose_group_by_index(
        scan_pose_groups,
        normalized_pose_index,
        current_scan_pose_group
    );

    BindExecutionPathOriginPose merged_path_origin = path_origin;
    nlohmann::json flattened_areas =
        flatten_scan_pose_groups_bind_path(scan_pose_groups, &merged_path_origin);
    if (merged_path_origin_out != nullptr) {
        *merged_path_origin_out = merged_path_origin;
    }

    nlohmann::json bind_path_json;
    bind_path_json["scan_session_id"] = scan_session_id;
    bind_path_json["path_signature"] = path_signature;
    bind_path_json["scan_mode"] = "scan_only";
    bind_path_json["cabin_height"] = cabin_height;
    bind_path_json["cabin_speed"] = cabin_speed;
    bind_path_json["path_origin"] = {
        {"x", merged_path_origin.x},
        {"y", merged_path_origin.y},
        {"z", merged_path_origin.z},
    };
    bind_path_json["scan_pose_groups"] = scan_pose_groups;
    bind_path_json["scan_pose_groups_by_pose_index"] =
        build_scan_pose_groups_by_pose_index(scan_pose_groups);
    bind_path_json["scan_pose_group_count"] = scan_pose_groups.size();
    bind_path_json["areas"] = flattened_areas;

    return write_json_file_atomically(pseudo_slam_bind_path_json_file, bind_path_json, error_message);
}

bool invalidate_scan_session_in_artifact_file(
    const std::string& artifact_path,
    const std::string& artifact_name,
    const std::string& invalid_reason,
    std::string* error_message
)
{
    if (error_message != nullptr) {
        error_message->clear();
    }

    std::ifstream artifact_file(artifact_path);
    if (!artifact_file.is_open()) {
        if (error_message != nullptr) {
            *error_message = artifact_name + "不存在，无法失效化scan_session_id";
        }
        return false;
    }

    nlohmann::json artifact_json;
    try {
        artifact_file >> artifact_json;
        if (artifact_file.fail() && !artifact_file.eof()) {
            throw std::runtime_error("artifact read failure");
        }
        if (!artifact_json.is_object()) {
            throw std::runtime_error("artifact root must be object");
        }
    } catch (const std::exception&) {
        if (error_message != nullptr) {
            *error_message = artifact_name + "读取或解析失败，无法失效化scan_session_id";
        }
        return false;
    }

    artifact_json["scan_session_id"] = "";
    artifact_json["scan_session_invalid_reason"] = invalid_reason;
    artifact_json["requires_rescan"] = true;

    std::string write_error;
    if (!write_json_file_atomically(artifact_path, artifact_json, &write_error)) {
        if (error_message != nullptr) {
            *error_message = artifact_name + "写回失败：" + write_error;
        }
        return false;
    }

    return true;
}

bool invalidate_current_scan_artifacts_after_execution_memory_write_failure(
    const std::string& write_failure_reason,
    std::string* error_message
)
{
    if (error_message != nullptr) {
        error_message->clear();
    }

    const std::string invalid_reason =
        "bind_execution_memory.json写入失败：" + write_failure_reason +
        "；为避免重启后继续消费旧账本，需要重新扫描/重新建图";

    std::string points_error;
    const bool points_invalidated = invalidate_scan_session_in_artifact_file(
        pseudo_slam_points_json_file,
        "pseudo_slam_points.json",
        invalid_reason,
        &points_error
    );
    std::string bind_path_error;
    const bool bind_path_invalidated = invalidate_scan_session_in_artifact_file(
        pseudo_slam_bind_path_json_file,
        "pseudo_slam_bind_path.json",
        invalid_reason,
        &bind_path_error
    );
    if (points_invalidated && bind_path_invalidated) {
        return true;
    }

    std::ostringstream oss;
    oss << "账本写盘失败后失效化当前扫描产物session失败";
    if (!points_invalidated) {
        oss << "；pseudo_slam_points.json=" << points_error;
    }
    if (!bind_path_invalidated) {
        oss << "；pseudo_slam_bind_path.json=" << bind_path_error;
    }
    if (error_message != nullptr) {
        *error_message = oss.str();
    }
    return false;
}
