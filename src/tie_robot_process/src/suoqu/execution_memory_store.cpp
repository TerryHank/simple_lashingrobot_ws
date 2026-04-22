#include "suoqu_runtime_internal.hpp"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <unistd.h>

bool load_bind_execution_memory_json(
    BindExecutionMemory& memory,
    std::string& error_message
)
{
    memory = BindExecutionMemory{};
    error_message.clear();
    std::ifstream file_obj(kBindExecutionMemoryJsonPath);
    if (!file_obj.is_open()) {
        if (access(kBindExecutionMemoryJsonPath.c_str(), F_OK) == 0) {
            printCurrentTime();
            printf("Cabin_log: bind_execution_memory.json读取或解析失败，无法打开现有记忆文件。\n");
            error_message = kBindExecutionMemoryUnreadableError;
            return false;
        }
        return true;
    }

    try {
        nlohmann::json memory_json;
        file_obj >> memory_json;
        if (file_obj.fail() && !file_obj.eof()) {
            throw std::runtime_error("bind execution memory read failure");
        }
        if (!memory_json.is_object()) {
            throw std::runtime_error("bind execution memory root must be object");
        }
        if (!memory_json.contains("executed_points")) {
            throw std::runtime_error("bind execution memory missing executed_points");
        }
        if (!memory_json["executed_points"].is_array()) {
            throw std::runtime_error("bind execution memory executed_points must be array");
        }

        memory.scan_session_id = memory_json.value("scan_session_id", "");
        memory.path_signature = memory_json.value("path_signature", "");
        if (memory_json.contains("path_origin") && !memory_json["path_origin"].is_object()) {
            throw std::runtime_error("bind execution memory path_origin must be object");
        }
        if (memory_json.contains("path_origin") && memory_json["path_origin"].is_object()) {
            const auto& path_origin_json = memory_json["path_origin"];
            memory.path_origin.x = path_origin_json.value("x", 0.0f);
            memory.path_origin.y = path_origin_json.value("y", 0.0f);
        }

        for (const auto& point_json : memory_json["executed_points"]) {
            if (!point_json.is_object()) {
                throw std::runtime_error("bind execution memory point must be object");
            }
            if (!point_json.contains("global_row") || !point_json.contains("global_col")) {
                throw std::runtime_error("bind execution memory point missing global coordinates");
            }

            BindExecutionPointRecord point_record;
            point_record.global_row = point_json.at("global_row").get<int>();
            point_record.global_col = point_json.at("global_col").get<int>();
            point_record.checkerboard_parity = point_json.value("checkerboard_parity", -1);
            point_record.world_x = point_json.value("world_x", 0.0f);
            point_record.world_y = point_json.value("world_y", 0.0f);
            point_record.world_z = point_json.value("world_z", 0.0f);
            point_record.source_mode = point_json.value("source_mode", "");
            memory.executed_points.push_back(point_record);
        }
    } catch (const std::exception&) {
        printCurrentTime();
        printf("Cabin_log: bind_execution_memory.json读取、解析或语义校验失败，阻止执行以避免重复绑扎。\n");
        error_message = kBindExecutionMemoryUnreadableError;
        memory = BindExecutionMemory{};
        return false;
    }

    return true;
}

bool write_bind_execution_memory_json(const BindExecutionMemory& memory, std::string* error_message)
{
    if (error_message != nullptr) {
        error_message->clear();
    }

    nlohmann::json memory_json = {
        {"scan_session_id", memory.scan_session_id},
        {"path_signature", memory.path_signature},
        {"path_origin", {{"x", memory.path_origin.x}, {"y", memory.path_origin.y}, {"z", 0.0f}}},
        {"executed_points", nlohmann::json::array()},
    };
    for (const auto& point_record : memory.executed_points) {
        memory_json["executed_points"].push_back(
            {
                {"global_row", point_record.global_row},
                {"global_col", point_record.global_col},
                {"checkerboard_parity", point_record.checkerboard_parity},
                {"world_x", point_record.world_x},
                {"world_y", point_record.world_y},
                {"world_z", point_record.world_z},
                {"source_mode", point_record.source_mode},
            }
        );
    }

    const std::string temp_json_path = kBindExecutionMemoryJsonPath + ".tmp";
    std::ofstream file_obj(temp_json_path);
    if (!file_obj.is_open()) {
        if (error_message != nullptr) {
            *error_message = "无法打开bind_execution_memory.json临时文件进行写入";
        }
        return false;
    }

    file_obj << memory_json.dump(4);
    file_obj.flush();
    file_obj.close();
    if (!file_obj.good()) {
        std::remove(temp_json_path.c_str());
        if (error_message != nullptr) {
            *error_message = "写入bind_execution_memory.json临时文件失败";
        }
        return false;
    }
    if (std::rename(temp_json_path.c_str(), kBindExecutionMemoryJsonPath.c_str()) != 0) {
        std::remove(temp_json_path.c_str());
        if (error_message != nullptr) {
            *error_message = "原子替换bind_execution_memory.json失败";
        }
        return false;
    }
    return true;
}


BindExecutionMemory reset_bind_execution_memory_for_scan_session(
    const std::string& scan_session_id,
    const std::string& path_signature,
    const Cabin_Point& path_origin
)
{
    BindExecutionMemory memory;
    memory.scan_session_id = scan_session_id;
    memory.path_signature = path_signature;
    memory.path_origin = path_origin;
    memory.executed_points.clear();
    return memory;
}

bool reset_bind_execution_memory_from_current_scan_artifacts(
    const std::string& current_path_signature,
    std::string& error_message
)
{
    error_message.clear();

    nlohmann::json points_json;
    std::string points_error;
    if (!load_scan_artifact_json(
            pseudo_slam_points_json_file,
            "pseudo_slam_points.json",
            points_json,
            points_error
        )) {
        error_message = points_error;
        return false;
    }

    nlohmann::json bind_path_json;
    std::string bind_path_error;
    if (!load_scan_artifact_json(
            pseudo_slam_bind_path_json_file,
            "pseudo_slam_bind_path.json",
            bind_path_json,
            bind_path_error
        )) {
        error_message = bind_path_error;
        return false;
    }

    const std::string points_scan_session_id = points_json.value("scan_session_id", std::string());
    const std::string bind_path_scan_session_id = bind_path_json.value("scan_session_id", std::string());
    if (points_scan_session_id.empty() ||
        bind_path_scan_session_id.empty() ||
        points_scan_session_id != bind_path_scan_session_id) {
        error_message = "扫描产物scan_session_id不一致，无法按请求清空执行记忆，请先重新扫描建图";
        return false;
    }

    const std::string points_path_signature = points_json.value("path_signature", std::string());
    const std::string bind_path_signature = bind_path_json.value("path_signature", std::string());
    if (points_path_signature.empty() ||
        bind_path_signature.empty() ||
        points_path_signature != bind_path_signature ||
        bind_path_signature != current_path_signature) {
        error_message = "扫描产物path_signature与当前路径不一致，无法按请求清空执行记忆，请先重新扫描建图";
        return false;
    }

    Cabin_Point path_origin{};
    if (bind_path_json.contains("path_origin") && bind_path_json["path_origin"].is_object()) {
        path_origin.x = bind_path_json["path_origin"].value("x", 0.0f);
        path_origin.y = bind_path_json["path_origin"].value("y", 0.0f);
    } else if (bind_path_json.contains("areas") &&
               bind_path_json["areas"].is_array() &&
               !bind_path_json["areas"].empty() &&
               bind_path_json["areas"].front().contains("cabin_pose")) {
        const auto& first_cabin_pose = bind_path_json["areas"].front()["cabin_pose"];
        path_origin.x = first_cabin_pose.value("x", 0.0f);
        path_origin.y = first_cabin_pose.value("y", 0.0f);
    } else {
        error_message = "pseudo_slam_bind_path.json缺少path_origin，无法按请求清空执行记忆";
        return false;
    }

    BindExecutionMemory bind_execution_memory =
        reset_bind_execution_memory_for_scan_session(
            bind_path_scan_session_id,
            current_path_signature,
            path_origin
        );
    std::string bind_execution_memory_error;
    if (!write_bind_execution_memory_json(bind_execution_memory, &bind_execution_memory_error)) {
        error_message = "按请求清空bind_execution_memory.json失败：" + bind_execution_memory_error;
        return false;
    }

    printCurrentTime();
    printf(
        "Cabin_log: 已按请求清空bind_execution_memory.json，scan_session_id=%s，path_signature=%s。\n",
        bind_path_scan_session_id.c_str(),
        current_path_signature.c_str()
    );
    return true;
}

bool is_point_already_executed(
    const BindExecutionMemory& memory,
    int global_row,
    int global_col
)
{
    if (global_row < 0 || global_col < 0) {
        return false;
    }

    return std::any_of(
        memory.executed_points.begin(),
        memory.executed_points.end(),
        [global_row, global_col](const BindExecutionPointRecord& point_record) {
            return point_record.global_row == global_row && point_record.global_col == global_col;
        }
    );
}


void record_successful_execution_point(
    BindExecutionMemory& memory,
    const nlohmann::json& point_json,
    const std::string& source_mode
)
{
    const int global_row = point_json.value("global_row", -1);
    const int global_col = point_json.value("global_col", -1);
    if (global_row < 0 || global_col < 0) {
        return;
    }
    if (is_point_already_executed(memory, global_row, global_col)) {
        return;
    }

    BindExecutionPointRecord point_record;
    point_record.global_row = global_row;
    point_record.global_col = global_col;
    point_record.checkerboard_parity = point_json.value("checkerboard_parity", -1);
    point_record.world_x = point_json.value("world_x", point_json.value("x", 0.0f));
    point_record.world_y = point_json.value("world_y", point_json.value("y", 0.0f));
    point_record.world_z = point_json.value("world_z", point_json.value("z", 0.0f));
    point_record.source_mode = source_mode;
    memory.executed_points.push_back(point_record);
}
