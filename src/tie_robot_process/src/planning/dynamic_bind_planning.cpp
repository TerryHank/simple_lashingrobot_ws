#include "dynamic_bind_planning_internal.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <unordered_set>
#include <unordered_map>
#include <utility>

namespace tie_robot_process {
namespace planning {

namespace {

long long encode_grid_cell_key(int row_index, int column_index)
{
    return (static_cast<long long>(row_index) << 32) ^
           static_cast<unsigned int>(column_index);
}

float get_grid_axis_cluster_threshold(const DynamicBindPlannerConfig& config, int axis_index)
{
    if (axis_index == 0) {
        return config.matrix_column_threshold_mm;
    }
    return config.matrix_row_threshold_mm;
}

std::vector<float> cluster_world_axis_centers(
    const std::vector<tie_robot_msgs::PointCoords>& world_points,
    int axis_index,
    float threshold_mm)
{
    std::vector<float> axis_values;
    axis_values.reserve(world_points.size());
    for (const auto& world_point : world_points) {
        axis_values.push_back(world_point.World_coord[axis_index]);
    }
    if (axis_values.empty()) {
        return {};
    }

    std::sort(axis_values.begin(), axis_values.end());

    std::vector<float> centers;
    std::vector<float> current_group = {axis_values.front()};
    float current_mean = axis_values.front();
    for (size_t value_index = 1; value_index < axis_values.size(); ++value_index) {
        const float axis_value = axis_values[value_index];
        if (std::fabs(axis_value - current_mean) <= threshold_mm) {
            current_group.push_back(axis_value);
            float axis_sum = 0.0f;
            for (const float grouped_value : current_group) {
                axis_sum += grouped_value;
            }
            current_mean = axis_sum / static_cast<float>(current_group.size());
            continue;
        }

        centers.push_back(current_mean);
        current_group = {axis_value};
        current_mean = axis_value;
    }
    centers.push_back(current_mean);
    return centers;
}

int find_nearest_axis_center_index(float axis_value, const std::vector<float>& centers)
{
    if (centers.empty()) {
        return -1;
    }

    int best_index = -1;
    float best_distance = std::numeric_limits<float>::max();
    for (size_t center_index = 0; center_index < centers.size(); ++center_index) {
        const float distance = std::fabs(axis_value - centers[center_index]);
        if (best_index < 0 || distance < best_distance) {
            best_index = static_cast<int>(center_index);
            best_distance = distance;
        }
    }
    return best_index;
}

struct GridPointRef
{
    int row_index = -1;
    int column_index = -1;
    const tie_robot_msgs::PointCoords* point = nullptr;
};

struct GridSquareCandidate
{
    int row_index = -1;
    int column_index = -1;
    int traversal_order = 0;
    double geometry_score = 0.0;
    std::vector<GridPointRef> refs;
};

struct CandidateWorldBounds
{
    bool valid = false;
    double min_x = 0.0;
    double min_y = 0.0;
    int min_idx = 0;
};

struct GridCellCoord
{
    int row_index = -1;
    int column_index = -1;
};

struct GridPairCandidate
{
    int row_index = -1;
    int column_index = -1;
    bool horizontal = false;
    double geometry_score = 0.0;
    std::vector<GridPointRef> refs;
};

struct GridRectangleShape
{
    int row_count = 1;
    int column_count = 1;
};

int normalize_requested_group_point_count(const DynamicBindPlannerConfig& config)
{
    return std::max(1, config.requested_group_point_count);
}

std::vector<GridRectangleShape> build_requested_group_rectangle_shapes(
    const DynamicBindPlannerConfig& config)
{
    const int requested_count = normalize_requested_group_point_count(config);
    std::vector<GridRectangleShape> shapes;
    for (int row_count = 1; row_count <= requested_count; ++row_count) {
        if (requested_count % row_count != 0) {
            continue;
        }
        const int column_count = requested_count / row_count;
        shapes.push_back(GridRectangleShape{row_count, column_count});
    }

    std::sort(shapes.begin(), shapes.end(), [](const GridRectangleShape& lhs, const GridRectangleShape& rhs) {
        const int lhs_squareness = std::abs(lhs.row_count - lhs.column_count);
        const int rhs_squareness = std::abs(rhs.row_count - rhs.column_count);
        if (lhs_squareness != rhs_squareness) {
            return lhs_squareness < rhs_squareness;
        }
        if (lhs.row_count != rhs.row_count) {
            return lhs.row_count < rhs.row_count;
        }
        return lhs.column_count < rhs.column_count;
    });
    return shapes;
}

CandidateWorldBounds compute_candidate_world_bounds(const std::vector<GridPointRef>& refs)
{
    CandidateWorldBounds bounds;
    for (const auto& ref : refs) {
        if (ref.point == nullptr) {
            continue;
        }
        const double world_x = static_cast<double>(ref.point->World_coord[0]);
        const double world_y = static_cast<double>(ref.point->World_coord[1]);
        if (!bounds.valid) {
            bounds.valid = true;
            bounds.min_x = world_x;
            bounds.min_y = world_y;
            bounds.min_idx = ref.point->idx;
            continue;
        }
        bounds.min_x = std::min(bounds.min_x, world_x);
        bounds.min_y = std::min(bounds.min_y, world_y);
        bounds.min_idx = std::min(bounds.min_idx, ref.point->idx);
    }
    return bounds;
}

double world_row_key_y(const GridSquareCandidate& candidate)
{
    const CandidateWorldBounds bounds = compute_candidate_world_bounds(candidate.refs);
    return bounds.valid ? bounds.min_y : 0.0;
}

bool compare_candidates_by_world_x(
    const GridSquareCandidate& lhs,
    const GridSquareCandidate& rhs,
    bool ascending_x)
{
    const CandidateWorldBounds lhs_bounds = compute_candidate_world_bounds(lhs.refs);
    const CandidateWorldBounds rhs_bounds = compute_candidate_world_bounds(rhs.refs);
    if (lhs_bounds.valid != rhs_bounds.valid) {
        return lhs_bounds.valid;
    }
    if (lhs_bounds.valid && rhs_bounds.valid) {
        if (std::fabs(lhs_bounds.min_x - rhs_bounds.min_x) > 1e-6) {
            return ascending_x
                ? lhs_bounds.min_x < rhs_bounds.min_x
                : lhs_bounds.min_x > rhs_bounds.min_x;
        }
        if (std::fabs(lhs_bounds.min_y - rhs_bounds.min_y) > 1e-6) {
            return lhs_bounds.min_y < rhs_bounds.min_y;
        }
        if (lhs_bounds.min_idx != rhs_bounds.min_idx) {
            return lhs_bounds.min_idx < rhs_bounds.min_idx;
        }
    }
    if (lhs.traversal_order != rhs.traversal_order) {
        return lhs.traversal_order < rhs.traversal_order;
    }
    if (lhs.row_index != rhs.row_index) {
        return lhs.row_index < rhs.row_index;
    }
    return lhs.column_index < rhs.column_index;
}

void sort_grid_square_candidates_by_world_x_snake(
    std::vector<GridSquareCandidate>& candidates,
    float row_tolerance_mm)
{
    if (candidates.empty()) {
        return;
    }

    std::sort(candidates.begin(), candidates.end(), [](const GridSquareCandidate& lhs, const GridSquareCandidate& rhs) {
        const CandidateWorldBounds lhs_bounds = compute_candidate_world_bounds(lhs.refs);
        const CandidateWorldBounds rhs_bounds = compute_candidate_world_bounds(rhs.refs);
        if (lhs_bounds.valid != rhs_bounds.valid) {
            return lhs_bounds.valid;
        }
        if (lhs_bounds.valid && rhs_bounds.valid) {
            if (std::fabs(lhs_bounds.min_y - rhs_bounds.min_y) > 1e-6) {
                return lhs_bounds.min_y < rhs_bounds.min_y;
            }
            if (std::fabs(lhs_bounds.min_x - rhs_bounds.min_x) > 1e-6) {
                return lhs_bounds.min_x < rhs_bounds.min_x;
            }
            if (lhs_bounds.min_idx != rhs_bounds.min_idx) {
                return lhs_bounds.min_idx < rhs_bounds.min_idx;
            }
        }
        if (lhs.traversal_order != rhs.traversal_order) {
            return lhs.traversal_order < rhs.traversal_order;
        }
        if (lhs.row_index != rhs.row_index) {
            return lhs.row_index < rhs.row_index;
        }
        return lhs.column_index < rhs.column_index;
    });

    std::vector<std::vector<GridSquareCandidate>> snake_rows;
    std::vector<double> row_mean_y_values;
    for (const auto& candidate : candidates) {
        const double candidate_row_y = world_row_key_y(candidate);
        if (snake_rows.empty() ||
            std::fabs(candidate_row_y - row_mean_y_values.back()) > static_cast<double>(row_tolerance_mm)) {
            snake_rows.push_back({candidate});
            row_mean_y_values.push_back(candidate_row_y);
            continue;
        }

        auto& row_candidates = snake_rows.back();
        row_candidates.push_back(candidate);
        row_mean_y_values.back() =
            (row_mean_y_values.back() * static_cast<double>(row_candidates.size() - 1U) +
             candidate_row_y) /
            static_cast<double>(row_candidates.size());
    }

    candidates.clear();
    for (size_t row_index = 0; row_index < snake_rows.size(); ++row_index) {
        auto& row_candidates = snake_rows[row_index];
        const bool ascending_x = (row_index % 2U) == 0U;
        std::sort(row_candidates.begin(), row_candidates.end(), [&](const GridSquareCandidate& lhs, const GridSquareCandidate& rhs) {
            return compare_candidates_by_world_x(lhs, rhs, ascending_x);
        });
        candidates.insert(candidates.end(), row_candidates.begin(), row_candidates.end());
    }
}

std::vector<GridPointRef> order_grid_refs_by_world_y_then_x(std::vector<GridPointRef> refs)
{
    std::sort(refs.begin(), refs.end(), [](const GridPointRef& lhs, const GridPointRef& rhs) {
        if (lhs.point == nullptr || rhs.point == nullptr) {
            return lhs.point != nullptr;
        }
        if (std::fabs(lhs.point->World_coord[1] - rhs.point->World_coord[1]) > 1e-6f) {
            return lhs.point->World_coord[1] < rhs.point->World_coord[1];
        }
        if (std::fabs(lhs.point->World_coord[0] - rhs.point->World_coord[0]) > 1e-6f) {
            return lhs.point->World_coord[0] < rhs.point->World_coord[0];
        }
        return lhs.point->idx < rhs.point->idx;
    });
    return refs;
}

bool are_adjacent_grid_refs(const GridPointRef& left, const GridPointRef& right)
{
    const int row_delta = std::abs(left.row_index - right.row_index);
    const int column_delta = std::abs(left.column_index - right.column_index);
    return (row_delta == 0 && column_delta == 1) ||
           (row_delta == 1 && column_delta == 0);
}

double square_candidate_geometry_score(
    const std::vector<GridPointRef>& refs,
    const DynamicBindPlannerConfig& config)
{
    if (refs.size() != 4U) {
        return std::numeric_limits<double>::max();
    }

    const auto axis = [](const GridPointRef& ref, int axis_index) -> double {
        if (ref.point == nullptr) {
            return 0.0;
        }
        return static_cast<double>(ref.point->World_coord[axis_index]);
    };

    const double nominal_spacing = std::max(
        1.0,
        static_cast<double>(config.nominal_grid_spacing_mm));
    const double top_row_alignment = std::fabs(axis(refs[0], 1) - axis(refs[1], 1));
    const double bottom_row_alignment = std::fabs(axis(refs[2], 1) - axis(refs[3], 1));
    const double left_column_alignment = std::fabs(axis(refs[0], 0) - axis(refs[2], 0));
    const double right_column_alignment = std::fabs(axis(refs[1], 0) - axis(refs[3], 0));
    const double top_column_gap = std::fabs(std::fabs(axis(refs[1], 0) - axis(refs[0], 0)) - nominal_spacing);
    const double bottom_column_gap = std::fabs(std::fabs(axis(refs[3], 0) - axis(refs[2], 0)) - nominal_spacing);
    const double left_row_gap = std::fabs(std::fabs(axis(refs[2], 1) - axis(refs[0], 1)) - nominal_spacing);
    const double right_row_gap = std::fabs(std::fabs(axis(refs[3], 1) - axis(refs[1], 1)) - nominal_spacing);
    const double z_spread =
        std::fabs(axis(refs[0], 2) - axis(refs[1], 2)) +
        std::fabs(axis(refs[2], 2) - axis(refs[3], 2));

    return 3.0 * (top_row_alignment + bottom_row_alignment + left_column_alignment + right_column_alignment) +
           top_column_gap + bottom_column_gap + left_row_gap + right_row_gap +
           0.25 * z_spread;
}

std::vector<GridPointRef> select_best_square_refs(
    const std::vector<GridPointRef>& top_left_refs,
    const std::vector<GridPointRef>& top_right_refs,
    const std::vector<GridPointRef>& bottom_left_refs,
    const std::vector<GridPointRef>& bottom_right_refs,
    const DynamicBindPlannerConfig& config,
    double& best_score)
{
    std::vector<GridPointRef> best_refs;
    best_score = std::numeric_limits<double>::max();

    for (const auto& top_left_ref : top_left_refs) {
        for (const auto& top_right_ref : top_right_refs) {
            for (const auto& bottom_left_ref : bottom_left_refs) {
                for (const auto& bottom_right_ref : bottom_right_refs) {
                    std::vector<GridPointRef> refs = {
                        top_left_ref,
                        top_right_ref,
                        bottom_left_ref,
                        bottom_right_ref,
                    };
                    const double score = square_candidate_geometry_score(refs, config);
                    const int index_sum =
                        (top_left_ref.point != nullptr ? top_left_ref.point->idx : 0) +
                        (top_right_ref.point != nullptr ? top_right_ref.point->idx : 0) +
                        (bottom_left_ref.point != nullptr ? bottom_left_ref.point->idx : 0) +
                        (bottom_right_ref.point != nullptr ? bottom_right_ref.point->idx : 0);
                    int best_index_sum = 0;
                    for (const auto& best_ref : best_refs) {
                        best_index_sum += best_ref.point != nullptr ? best_ref.point->idx : 0;
                    }
                    if (best_refs.empty() || score < best_score - 1e-6 ||
                        (std::fabs(score - best_score) <= 1e-6 && index_sum < best_index_sum)) {
                        best_score = score;
                        best_refs = refs;
                    }
                }
            }
        }
    }
    return best_refs;
}

int find_dense_index_for_key(const std::vector<int>& keys, int key)
{
    const auto key_it = std::find(keys.begin(), keys.end(), key);
    if (key_it == keys.end() || *key_it != key) {
        return -1;
    }
    return static_cast<int>(std::distance(keys.begin(), key_it));
}

std::vector<int> build_dense_keys_from_count(size_t count)
{
    std::vector<int> keys;
    keys.reserve(count);
    for (size_t key_index = 0; key_index < count; ++key_index) {
        keys.push_back(static_cast<int>(key_index));
    }
    return keys;
}

int build_grid_group_snake_traversal_order(int row_index, int column_index, int column_count)
{
    const int column_band_count = std::max((column_count + 1) / 2, 1);
    const int row_band_index = row_index / 2;
    const int column_band_index = column_index / 2;
    const bool moving_along_positive_x = (row_band_index % 2) == 0;
    const int column_order_index =
        moving_along_positive_x
            ? column_band_index
            : (column_band_count - 1 - column_band_index);
    return row_band_index * column_band_count + column_order_index;
}

int build_grid_rectangle_group_snake_traversal_order(
    int row_index,
    int column_index,
    int column_count,
    const GridRectangleShape& shape)
{
    const int safe_row_count = std::max(shape.row_count, 1);
    const int safe_column_count = std::max(shape.column_count, 1);
    const int column_band_count = std::max(
        (column_count + safe_column_count - 1) / safe_column_count,
        1);
    const int row_band_index = row_index / safe_row_count;
    const int column_band_index = column_index / safe_column_count;
    const bool moving_along_positive_x = (row_band_index % 2) == 0;
    const int column_order_index =
        moving_along_positive_x
            ? column_band_index
            : (column_band_count - 1 - column_band_index);
    return row_band_index * column_band_count + column_order_index;
}

std::vector<int> build_complete_non_negative_keys_to_max(const std::vector<int>& sparse_keys)
{
    if (sparse_keys.empty()) {
        return {};
    }
    const int max_key = *std::max_element(sparse_keys.begin(), sparse_keys.end());
    if (max_key < 0) {
        return {};
    }
    std::vector<int> keys;
    keys.reserve(static_cast<size_t>(max_key) + 1U);
    for (int key = 0; key <= max_key; ++key) {
        keys.push_back(key);
    }
    return keys;
}

std::vector<int> orient_grid_keys_from_world_minimum(
    std::vector<int> keys,
    const std::unordered_map<int, std::pair<int, int>>& grid_by_global_index,
    const std::unordered_map<int, const tie_robot_msgs::PointCoords*>& world_point_by_global_index,
    bool row_axis)
{
    if (keys.size() < 2U) {
        return keys;
    }

    struct AxisStats
    {
        double sum = 0.0;
        int count = 0;
    };
    std::unordered_map<int, AxisStats> stats_by_key;
    for (const auto& grid_entry : grid_by_global_index) {
        const auto point_it = world_point_by_global_index.find(grid_entry.first);
        if (point_it == world_point_by_global_index.end() || point_it->second == nullptr) {
            continue;
        }
        const int key = row_axis ? grid_entry.second.first : grid_entry.second.second;
        AxisStats& stats = stats_by_key[key];
        stats.sum += static_cast<double>(point_it->second->World_coord[row_axis ? 1 : 0]);
        stats.count++;
    }

    auto has_stats = [&](int key) {
        const auto stats_it = stats_by_key.find(key);
        return stats_it != stats_by_key.end() && stats_it->second.count > 0;
    };
    auto mean_for_key = [&](int key) {
        const AxisStats& stats = stats_by_key.at(key);
        return stats.sum / static_cast<double>(stats.count);
    };

    int first_observed_key = -1;
    int last_observed_key = -1;
    for (const int key : keys) {
        if (has_stats(key)) {
            first_observed_key = key;
            break;
        }
    }
    for (auto key_it = keys.rbegin(); key_it != keys.rend(); ++key_it) {
        if (has_stats(*key_it)) {
            last_observed_key = *key_it;
            break;
        }
    }
    if (first_observed_key < 0 ||
        last_observed_key < 0 ||
        first_observed_key == last_observed_key) {
        return keys;
    }

    if (mean_for_key(first_observed_key) > mean_for_key(last_observed_key) + 1e-6) {
        std::reverse(keys.begin(), keys.end());
    }
    return keys;
}

bool has_grid_cell_refs(
    const std::unordered_map<long long, std::vector<GridPointRef>>& point_refs_by_grid_cell,
    int row_index,
    int column_index)
{
    const auto cell_it = point_refs_by_grid_cell.find(encode_grid_cell_key(row_index, column_index));
    return cell_it != point_refs_by_grid_cell.end() && !cell_it->second.empty();
}

bool has_neighboring_grid_support(
    const std::unordered_map<long long, std::vector<GridPointRef>>& point_refs_by_grid_cell,
    int row_index,
    int column_index,
    int source_row_index,
    int source_column_index)
{
    constexpr std::array<std::pair<int, int>, 4> kNeighborOffsets{{
        {-1, 0},
        {1, 0},
        {0, -1},
        {0, 1},
    }};
    for (const auto& offset : kNeighborOffsets) {
        const int neighbor_row = row_index + offset.first;
        const int neighbor_column = column_index + offset.second;
        if (neighbor_row == source_row_index && neighbor_column == source_column_index) {
            continue;
        }
        if (has_grid_cell_refs(point_refs_by_grid_cell, neighbor_row, neighbor_column)) {
            return true;
        }
    }
    return false;
}

size_t select_duplicate_ref_for_empty_neighbor(
    const std::vector<GridPointRef>& refs,
    int source_row_index,
    int source_column_index,
    int target_row_index,
    int target_column_index)
{
    size_t selected_index = 0;
    double selected_axis_value = 0.0;
    bool selected = false;
    const bool choose_larger =
        target_row_index > source_row_index || target_column_index > source_column_index;
    const int axis_index = target_row_index != source_row_index ? 1 : 0;
    for (size_t ref_index = 0; ref_index < refs.size(); ++ref_index) {
        if (refs[ref_index].point == nullptr) {
            continue;
        }
        const double axis_value = static_cast<double>(refs[ref_index].point->World_coord[axis_index]);
        if (!selected ||
            (choose_larger && axis_value > selected_axis_value) ||
            (!choose_larger && axis_value < selected_axis_value)) {
            selected = true;
            selected_index = ref_index;
            selected_axis_value = axis_value;
        }
    }
    return selected_index;
}

void relocate_duplicate_grid_refs_to_adjacent_gaps(
    std::unordered_map<long long, std::vector<GridPointRef>>& point_refs_by_grid_cell,
    int row_count,
    int column_count)
{
    bool changed = true;
    while (changed) {
        changed = false;
        for (int row_index = 0; row_index < row_count && !changed; ++row_index) {
            for (int column_index = 0; column_index < column_count && !changed; ++column_index) {
                const long long source_key = encode_grid_cell_key(row_index, column_index);
                auto source_it = point_refs_by_grid_cell.find(source_key);
                if (source_it == point_refs_by_grid_cell.end() || source_it->second.size() <= 1U) {
                    continue;
                }

                constexpr std::array<std::pair<int, int>, 4> kRepairOffsets{{
                    {1, 0},
                    {-1, 0},
                    {0, 1},
                    {0, -1},
                }};
                for (const auto& offset : kRepairOffsets) {
                    const int target_row = row_index + offset.first;
                    const int target_column = column_index + offset.second;
                    if (target_row < 0 ||
                        target_column < 0 ||
                        target_row >= row_count ||
                        target_column >= column_count ||
                        has_grid_cell_refs(point_refs_by_grid_cell, target_row, target_column) ||
                        !has_neighboring_grid_support(
                            point_refs_by_grid_cell,
                            target_row,
                            target_column,
                            row_index,
                            column_index)) {
                        continue;
                    }

                    const size_t selected_ref_index = select_duplicate_ref_for_empty_neighbor(
                        source_it->second,
                        row_index,
                        column_index,
                        target_row,
                        target_column);
                    GridPointRef moved_ref = source_it->second[selected_ref_index];
                    moved_ref.row_index = target_row;
                    moved_ref.column_index = target_column;
                    source_it->second.erase(source_it->second.begin() + static_cast<long>(selected_ref_index));
                    point_refs_by_grid_cell[encode_grid_cell_key(target_row, target_column)].push_back(moved_ref);
                    changed = true;
                    break;
                }
            }
        }
    }
}

double pair_candidate_geometry_score(
    const GridPointRef& first_ref,
    const GridPointRef& second_ref,
    const DynamicBindPlannerConfig& config)
{
    if (first_ref.point == nullptr || second_ref.point == nullptr) {
        return std::numeric_limits<double>::max();
    }

    const int row_delta = std::abs(first_ref.row_index - second_ref.row_index);
    const int column_delta = std::abs(first_ref.column_index - second_ref.column_index);
    const double nominal_spacing = std::max(
        1.0,
        static_cast<double>(config.nominal_grid_spacing_mm));
    const double dx = static_cast<double>(second_ref.point->World_coord[0]) -
                      static_cast<double>(first_ref.point->World_coord[0]);
    const double dy = static_cast<double>(second_ref.point->World_coord[1]) -
                      static_cast<double>(first_ref.point->World_coord[1]);
    const double dz = static_cast<double>(second_ref.point->World_coord[2]) -
                      static_cast<double>(first_ref.point->World_coord[2]);
    const double spacing_error =
        std::fabs(std::sqrt(dx * dx + dy * dy) - nominal_spacing);
    return spacing_error + 0.25 * std::fabs(dz) + 1000.0 * std::abs(row_delta + column_delta - 1);
}

std::vector<GridPointRef> select_best_pair_refs(
    const std::vector<GridPointRef>& first_refs,
    const std::vector<GridPointRef>& second_refs,
    const DynamicBindPlannerConfig& config,
    double& best_score)
{
    std::vector<GridPointRef> best_refs;
    best_score = std::numeric_limits<double>::max();

    for (const auto& first_ref : first_refs) {
        for (const auto& second_ref : second_refs) {
            const double score = pair_candidate_geometry_score(first_ref, second_ref, config);
            const int index_sum =
                (first_ref.point != nullptr ? first_ref.point->idx : 0) +
                (second_ref.point != nullptr ? second_ref.point->idx : 0);
            int best_index_sum = 0;
            for (const auto& best_ref : best_refs) {
                best_index_sum += best_ref.point != nullptr ? best_ref.point->idx : 0;
            }
            if (best_refs.empty() || score < best_score - 1e-6 ||
                (std::fabs(score - best_score) <= 1e-6 && index_sum < best_index_sum)) {
                best_score = score;
                best_refs = {first_ref, second_ref};
            }
        }
    }
    return best_refs;
}

std::vector<GridSquareCandidate> select_grid_group_candidates_by_fixed_two_by_two_tiling(
    const std::unordered_map<long long, std::vector<GridPointRef>>& point_refs_by_grid_cell,
    int row_count,
    int column_count,
    const DynamicBindPlannerConfig& config)
{
    std::vector<GridSquareCandidate> selected_candidates;
    if (row_count <= 0 || column_count <= 0) {
        return selected_candidates;
    }

    auto find_cell_refs = [&](int row_index, int column_index) -> const std::vector<GridPointRef>* {
        const auto cell_it = point_refs_by_grid_cell.find(encode_grid_cell_key(row_index, column_index));
        if (cell_it == point_refs_by_grid_cell.end() || cell_it->second.empty()) {
            return nullptr;
        }
        return &cell_it->second;
    };

    auto push_candidate = [&](int row_index, int column_index, double geometry_score, std::vector<GridPointRef> refs) {
        if (refs.size() != 2U && refs.size() != 4U) {
            return;
        }
        selected_candidates.push_back(GridSquareCandidate{
            row_index,
            column_index,
            build_grid_group_snake_traversal_order(row_index, column_index, column_count),
            geometry_score,
            std::move(refs),
        });
    };

    for (int row_index = 0; row_index < row_count; row_index += 2) {
        for (int column_index = 0; column_index < column_count; column_index += 2) {
            const bool has_next_row = row_index + 1 < row_count;
            const bool has_next_column = column_index + 1 < column_count;
            if (has_next_row && has_next_column) {
                const auto* top_left_refs = find_cell_refs(row_index, column_index);
                const auto* top_right_refs = find_cell_refs(row_index, column_index + 1);
                const auto* bottom_left_refs = find_cell_refs(row_index + 1, column_index);
                const auto* bottom_right_refs = find_cell_refs(row_index + 1, column_index + 1);
                if (top_left_refs == nullptr ||
                    top_right_refs == nullptr ||
                    bottom_left_refs == nullptr ||
                    bottom_right_refs == nullptr) {
                    continue;
                }

                double geometry_score = 0.0;
                std::vector<GridPointRef> refs = select_best_square_refs(
                    *top_left_refs,
                    *top_right_refs,
                    *bottom_left_refs,
                    *bottom_right_refs,
                    config,
                    geometry_score);
                push_candidate(row_index, column_index, geometry_score, std::move(refs));
                continue;
            }

            if (!has_next_row && has_next_column) {
                const auto* left_refs = find_cell_refs(row_index, column_index);
                const auto* right_refs = find_cell_refs(row_index, column_index + 1);
                if (left_refs == nullptr || right_refs == nullptr) {
                    continue;
                }
                double geometry_score = 0.0;
                std::vector<GridPointRef> refs = select_best_pair_refs(
                    *left_refs,
                    *right_refs,
                    config,
                    geometry_score);
                push_candidate(row_index, column_index, geometry_score, std::move(refs));
                continue;
            }

            if (has_next_row && !has_next_column) {
                const auto* top_refs = find_cell_refs(row_index, column_index);
                const auto* bottom_refs = find_cell_refs(row_index + 1, column_index);
                if (top_refs == nullptr || bottom_refs == nullptr) {
                    continue;
                }
                double geometry_score = 0.0;
                std::vector<GridPointRef> refs = select_best_pair_refs(
                    *top_refs,
                    *bottom_refs,
                    config,
                    geometry_score);
                push_candidate(row_index, column_index, geometry_score, std::move(refs));
            }
        }
    }

    return selected_candidates;
}

std::vector<GridSquareCandidate> select_grid_group_candidates_by_requested_rectangle_tiling(
    const std::unordered_map<long long, std::vector<GridPointRef>>& point_refs_by_grid_cell,
    int row_count,
    int column_count,
    const DynamicBindPlannerConfig& config)
{
    std::vector<GridSquareCandidate> selected_candidates;
    if (row_count <= 0 || column_count <= 0) {
        return selected_candidates;
    }

    auto find_cell_refs = [&](int row_index, int column_index) -> const std::vector<GridPointRef>* {
        const auto cell_it = point_refs_by_grid_cell.find(encode_grid_cell_key(row_index, column_index));
        if (cell_it == point_refs_by_grid_cell.end() || cell_it->second.empty()) {
            return nullptr;
        }
        return &cell_it->second;
    };

    const std::vector<GridRectangleShape> shapes =
        build_requested_group_rectangle_shapes(config);
    for (const auto& shape : shapes) {
        if (shape.row_count <= 0 ||
            shape.column_count <= 0 ||
            shape.row_count > row_count ||
            shape.column_count > column_count) {
            continue;
        }
        for (int row_index = 0; row_index + shape.row_count <= row_count; row_index += shape.row_count) {
            for (int column_index = 0; column_index + shape.column_count <= column_count; column_index += shape.column_count) {
                std::vector<GridPointRef> refs;
                refs.reserve(static_cast<size_t>(shape.row_count * shape.column_count));
                bool complete_rectangle = true;
                for (int local_row = 0; local_row < shape.row_count && complete_rectangle; ++local_row) {
                    for (int local_column = 0; local_column < shape.column_count; ++local_column) {
                        const auto* cell_refs = find_cell_refs(
                            row_index + local_row,
                            column_index + local_column);
                        if (cell_refs == nullptr) {
                            complete_rectangle = false;
                            break;
                        }
                        refs.push_back(cell_refs->front());
                    }
                }
                if (!complete_rectangle ||
                    static_cast<int>(refs.size()) != shape.row_count * shape.column_count) {
                    continue;
                }

                selected_candidates.push_back(GridSquareCandidate{
                    row_index,
                    column_index,
                    build_grid_rectangle_group_snake_traversal_order(
                        row_index,
                        column_index,
                        column_count,
                        shape),
                    0.0,
                    std::move(refs),
                });
            }
        }
    }

    return selected_candidates;
}

std::vector<GridSquareCandidate> select_grid_group_candidates_by_adjacency_matching(
    const std::unordered_map<long long, std::vector<GridPointRef>>& point_refs_by_grid_cell,
    int row_count,
    int column_count,
    const DynamicBindPlannerConfig& config)
{
    std::vector<GridSquareCandidate> selected_candidates;
    if (row_count <= 0 || column_count <= 0) {
        return selected_candidates;
    }

    auto find_cell_refs = [&](int row_index, int column_index) -> const std::vector<GridPointRef>* {
        const auto cell_it = point_refs_by_grid_cell.find(encode_grid_cell_key(row_index, column_index));
        if (cell_it == point_refs_by_grid_cell.end() || cell_it->second.empty()) {
            return nullptr;
        }
        return &cell_it->second;
    };

    std::vector<GridCellCoord> left_cells;
    std::unordered_set<long long> occupied_cell_keys;
    for (int row_index = 0; row_index < row_count; ++row_index) {
        for (int column_index = 0; column_index < column_count; ++column_index) {
            if (find_cell_refs(row_index, column_index) == nullptr) {
                continue;
            }
            occupied_cell_keys.insert(encode_grid_cell_key(row_index, column_index));
            if (((row_index + column_index) % 2) == 0) {
                left_cells.push_back(GridCellCoord{row_index, column_index});
            }
        }
    }
    if (occupied_cell_keys.size() < 2U || left_cells.empty()) {
        return selected_candidates;
    }

    struct NeighborOption
    {
        GridCellCoord cell;
        double geometry_score = std::numeric_limits<double>::max();
        int traversal_tiebreaker = 0;
    };
    std::unordered_map<long long, std::vector<NeighborOption>> neighbors_by_left_cell;
    constexpr std::array<std::pair<int, int>, 4> kNeighborOffsets{{
        {0, 1},
        {1, 0},
        {0, -1},
        {-1, 0},
    }};
    for (const auto& left_cell : left_cells) {
        std::vector<NeighborOption> neighbor_options;
        const auto* left_refs = find_cell_refs(left_cell.row_index, left_cell.column_index);
        if (left_refs == nullptr) {
            continue;
        }
        for (size_t offset_index = 0; offset_index < kNeighborOffsets.size(); ++offset_index) {
            const int neighbor_row = left_cell.row_index + kNeighborOffsets[offset_index].first;
            const int neighbor_column = left_cell.column_index + kNeighborOffsets[offset_index].second;
            if (neighbor_row < 0 ||
                neighbor_column < 0 ||
                neighbor_row >= row_count ||
                neighbor_column >= column_count ||
                occupied_cell_keys.count(encode_grid_cell_key(neighbor_row, neighbor_column)) == 0U) {
                continue;
            }

            const auto* neighbor_refs = find_cell_refs(neighbor_row, neighbor_column);
            if (neighbor_refs == nullptr) {
                continue;
            }
            double geometry_score = 0.0;
            (void)select_best_pair_refs(*left_refs, *neighbor_refs, config, geometry_score);
            neighbor_options.push_back(NeighborOption{
                GridCellCoord{neighbor_row, neighbor_column},
                geometry_score,
                static_cast<int>(offset_index),
            });
        }
        std::sort(neighbor_options.begin(), neighbor_options.end(), [](const NeighborOption& lhs, const NeighborOption& rhs) {
            if (std::fabs(lhs.geometry_score - rhs.geometry_score) > 1e-6) {
                return lhs.geometry_score < rhs.geometry_score;
            }
            if (lhs.traversal_tiebreaker != rhs.traversal_tiebreaker) {
                return lhs.traversal_tiebreaker < rhs.traversal_tiebreaker;
            }
            if (lhs.cell.row_index != rhs.cell.row_index) {
                return lhs.cell.row_index < rhs.cell.row_index;
            }
            return lhs.cell.column_index < rhs.cell.column_index;
        });
        neighbors_by_left_cell[encode_grid_cell_key(left_cell.row_index, left_cell.column_index)] =
            std::move(neighbor_options);
    }

    std::sort(left_cells.begin(), left_cells.end(), [&](const GridCellCoord& lhs, const GridCellCoord& rhs) {
        const auto lhs_neighbors_it = neighbors_by_left_cell.find(encode_grid_cell_key(lhs.row_index, lhs.column_index));
        const auto rhs_neighbors_it = neighbors_by_left_cell.find(encode_grid_cell_key(rhs.row_index, rhs.column_index));
        const size_t lhs_degree =
            lhs_neighbors_it == neighbors_by_left_cell.end() ? 0U : lhs_neighbors_it->second.size();
        const size_t rhs_degree =
            rhs_neighbors_it == neighbors_by_left_cell.end() ? 0U : rhs_neighbors_it->second.size();
        if (lhs_degree != rhs_degree) {
            return lhs_degree < rhs_degree;
        }
        if (lhs.row_index != rhs.row_index) {
            return lhs.row_index < rhs.row_index;
        }
        return lhs.column_index < rhs.column_index;
    });

    std::unordered_map<long long, GridCellCoord> matched_left_by_right_cell;
    std::function<bool(const GridCellCoord&, std::unordered_set<long long>&)> try_match_left_cell =
        [&](const GridCellCoord& left_cell, std::unordered_set<long long>& visited_right_cells) -> bool {
            const long long left_key = encode_grid_cell_key(left_cell.row_index, left_cell.column_index);
            const auto neighbors_it = neighbors_by_left_cell.find(left_key);
            if (neighbors_it == neighbors_by_left_cell.end()) {
                return false;
            }
            for (const auto& neighbor : neighbors_it->second) {
                const long long right_key = encode_grid_cell_key(neighbor.cell.row_index, neighbor.cell.column_index);
                if (!visited_right_cells.insert(right_key).second) {
                    continue;
                }
                const auto match_it = matched_left_by_right_cell.find(right_key);
                if (match_it == matched_left_by_right_cell.end() ||
                    try_match_left_cell(match_it->second, visited_right_cells)) {
                    matched_left_by_right_cell[right_key] = left_cell;
                    return true;
                }
            }
            return false;
        };

    for (const auto& left_cell : left_cells) {
        std::unordered_set<long long> visited_right_cells;
        (void)try_match_left_cell(left_cell, visited_right_cells);
    }

    std::vector<GridPairCandidate> matched_pairs;
    for (const auto& match_entry : matched_left_by_right_cell) {
        const GridCellCoord left_cell = match_entry.second;
        const GridCellCoord right_cell{
            static_cast<int>(match_entry.first >> 32),
            static_cast<int>(match_entry.first & 0xffffffff),
        };
        GridCellCoord first_cell = left_cell;
        GridCellCoord second_cell = right_cell;
        if (left_cell.row_index == right_cell.row_index) {
            if (right_cell.column_index < left_cell.column_index) {
                first_cell = right_cell;
                second_cell = left_cell;
            }
        } else if (right_cell.row_index < left_cell.row_index) {
            first_cell = right_cell;
            second_cell = left_cell;
        }

        const auto* first_refs = find_cell_refs(first_cell.row_index, first_cell.column_index);
        const auto* second_refs = find_cell_refs(second_cell.row_index, second_cell.column_index);
        if (first_refs == nullptr || second_refs == nullptr) {
            continue;
        }

        double geometry_score = 0.0;
        std::vector<GridPointRef> refs = select_best_pair_refs(*first_refs, *second_refs, config, geometry_score);
        if (refs.size() != 2U) {
            continue;
        }
        matched_pairs.push_back(GridPairCandidate{
            std::min(first_cell.row_index, second_cell.row_index),
            std::min(first_cell.column_index, second_cell.column_index),
            first_cell.row_index == second_cell.row_index,
            geometry_score,
            std::move(refs),
        });
    }

    std::sort(matched_pairs.begin(), matched_pairs.end(), [](const GridPairCandidate& lhs, const GridPairCandidate& rhs) {
        if (lhs.column_index != rhs.column_index) {
            return lhs.column_index < rhs.column_index;
        }
        if (lhs.row_index != rhs.row_index) {
            return lhs.row_index < rhs.row_index;
        }
        if (lhs.horizontal != rhs.horizontal) {
            return lhs.horizontal && !rhs.horizontal;
        }
        return lhs.geometry_score < rhs.geometry_score;
    });

    int traversal_order = 0;
    auto push_candidate = [&](int row_index, int column_index, double geometry_score, std::vector<GridPointRef> refs) {
        if (refs.empty()) {
            return;
        }
        selected_candidates.push_back(GridSquareCandidate{
            row_index,
            column_index,
            traversal_order++,
            geometry_score,
            std::move(refs),
        });
    };

    std::vector<bool> used_pairs(matched_pairs.size(), false);
    auto square_from_pair_indices = [&](size_t first_pair_index, size_t second_pair_index, double& geometry_score) {
        const auto& first_pair = matched_pairs[first_pair_index];
        const auto& second_pair = matched_pairs[second_pair_index];
        int top_row = -1;
        int left_column = -1;
        if (first_pair.horizontal &&
            second_pair.horizontal &&
            first_pair.column_index == second_pair.column_index &&
            std::abs(first_pair.row_index - second_pair.row_index) == 1) {
            top_row = std::min(first_pair.row_index, second_pair.row_index);
            left_column = first_pair.column_index;
        } else if (!first_pair.horizontal &&
                   !second_pair.horizontal &&
                   first_pair.row_index == second_pair.row_index &&
                   std::abs(first_pair.column_index - second_pair.column_index) == 1) {
            top_row = first_pair.row_index;
            left_column = std::min(first_pair.column_index, second_pair.column_index);
        } else {
            return std::vector<GridPointRef>{};
        }

        const auto* top_left_refs = find_cell_refs(top_row, left_column);
        const auto* top_right_refs = find_cell_refs(top_row, left_column + 1);
        const auto* bottom_left_refs = find_cell_refs(top_row + 1, left_column);
        const auto* bottom_right_refs = find_cell_refs(top_row + 1, left_column + 1);
        if (top_left_refs == nullptr ||
            top_right_refs == nullptr ||
            bottom_left_refs == nullptr ||
            bottom_right_refs == nullptr) {
            return std::vector<GridPointRef>{};
        }
        return select_best_square_refs(
            *top_left_refs,
            *top_right_refs,
            *bottom_left_refs,
            *bottom_right_refs,
            config,
            geometry_score);
    };

    for (size_t pair_index = 0; pair_index < matched_pairs.size(); ++pair_index) {
        if (used_pairs[pair_index]) {
            continue;
        }

        size_t merge_pair_index = matched_pairs.size();
        std::vector<GridPointRef> merged_square_refs;
        double merged_geometry_score = 0.0;
        for (size_t other_pair_index = pair_index + 1; other_pair_index < matched_pairs.size(); ++other_pair_index) {
            if (used_pairs[other_pair_index]) {
                continue;
            }
            double square_geometry_score = 0.0;
            std::vector<GridPointRef> square_refs =
                square_from_pair_indices(pair_index, other_pair_index, square_geometry_score);
            if (square_refs.size() != 4U) {
                continue;
            }
            if (merged_square_refs.empty() ||
                square_geometry_score < merged_geometry_score - 1e-6) {
                merge_pair_index = other_pair_index;
                merged_square_refs = std::move(square_refs);
                merged_geometry_score = square_geometry_score;
            }
        }
        if (!merged_square_refs.empty() && merge_pair_index < matched_pairs.size()) {
            used_pairs[pair_index] = true;
            used_pairs[merge_pair_index] = true;
            int square_row_index = std::min(matched_pairs[pair_index].row_index, matched_pairs[merge_pair_index].row_index);
            int square_column_index = std::min(matched_pairs[pair_index].column_index, matched_pairs[merge_pair_index].column_index);
            push_candidate(
                square_row_index,
                square_column_index,
                merged_geometry_score,
                std::move(merged_square_refs));
            continue;
        }

        used_pairs[pair_index] = true;
        push_candidate(
            matched_pairs[pair_index].row_index,
            matched_pairs[pair_index].column_index,
            matched_pairs[pair_index].geometry_score,
            matched_pairs[pair_index].refs);
    }

    return selected_candidates;
}

std::vector<tie_robot_msgs::PointCoords> collect_world_points_from_grid_refs(
    const std::vector<GridPointRef>& refs)
{
    std::vector<tie_robot_msgs::PointCoords> world_points;
    world_points.reserve(refs.size());
    for (const auto& ref : refs) {
        if (ref.point != nullptr) {
            world_points.push_back(*ref.point);
        }
    }
    return world_points;
}

bool is_group_reachable_from_centered_dynamic_pose(
    const std::vector<GridPointRef>& refs,
    float fallback_cabin_height,
    const tf2::Transform& gripper_from_base_link,
    const DynamicBindPlannerConfig& config)
{
    if (refs.empty()) {
        return false;
    }
    const std::vector<tie_robot_msgs::PointCoords> world_points =
        collect_world_points_from_grid_refs(refs);
    if (world_points.size() != refs.size()) {
        return false;
    }

    const internal::DynamicBindPlanningCandidatePose candidate_pose =
        internal::build_dynamic_bind_candidate_pose_from_world_point(
            world_points,
            gripper_from_base_link,
            config);
    const CabinPoint candidate_cabin_point{candidate_pose.cabin_x, candidate_pose.cabin_y};
    const float candidate_cabin_z = internal::clamp_bind_execution_cabin_z(
        candidate_pose.cabin_z > 0.0f ? candidate_pose.cabin_z : fallback_cabin_height,
        config);
    for (const auto& world_point : world_points) {
        tie_robot_msgs::PointCoords local_point;
        internal::transform_cabin_world_point_to_planned_gripper_point(
            world_point,
            candidate_cabin_point,
            candidate_cabin_z,
            gripper_from_base_link,
            local_point);
        if (!internal::is_local_bind_point_in_range(local_point, config)) {
            return false;
        }
    }
    return true;
}

std::pair<int, int> compute_group_grid_span(const std::vector<GridPointRef>& refs)
{
    if (refs.empty()) {
        return {0, 0};
    }
    int min_row = refs.front().row_index;
    int max_row = refs.front().row_index;
    int min_column = refs.front().column_index;
    int max_column = refs.front().column_index;
    for (const auto& ref : refs) {
        min_row = std::min(min_row, ref.row_index);
        max_row = std::max(max_row, ref.row_index);
        min_column = std::min(min_column, ref.column_index);
        max_column = std::max(max_column, ref.column_index);
    }
    return {max_row - min_row + 1, max_column - min_column + 1};
}

std::string build_bind_group_type_from_refs(
    const std::vector<GridPointRef>& refs,
    const DynamicBindPlannerConfig& config)
{
    const std::pair<int, int> span = compute_group_grid_span(refs);
    if (refs.size() == 2U && normalize_requested_group_point_count(config) == 4) {
        return "matrix_2x2_edge_pair";
    }
    if (span.first > 0 &&
        span.second > 0 &&
        span.first * span.second == static_cast<int>(refs.size())) {
        return "matrix_" + std::to_string(span.first) + "x" + std::to_string(span.second);
    }
    return "matrix_custom_" + std::to_string(refs.size());
}

}  // namespace

float get_dynamic_bind_world_axis_value(
    const tie_robot_msgs::PointCoords& world_point,
    DynamicBindWorldAxis axis)
{
    return world_point.World_coord[axis == DynamicBindWorldAxis::kX ? 0 : 1];
}

DynamicBindGridAxisMapping infer_dynamic_bind_grid_axis_mapping(
    const std::vector<tie_robot_msgs::PointCoords>& planning_world_points,
    const std::vector<DynamicBindGridIndex>& grid_indices)
{
    struct AxisSpanAccumulator
    {
        int count = 0;
        float min_x = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float min_y = std::numeric_limits<float>::max();
        float max_y = std::numeric_limits<float>::lowest();

        void add(const tie_robot_msgs::PointCoords& point)
        {
            const float x = point.World_coord[0];
            const float y = point.World_coord[1];
            if (!std::isfinite(x) || !std::isfinite(y)) {
                return;
            }
            count++;
            min_x = std::min(min_x, x);
            max_x = std::max(max_x, x);
            min_y = std::min(min_y, y);
            max_y = std::max(max_y, y);
        }

        float span_x() const
        {
            return count > 0 ? max_x - min_x : 0.0f;
        }

        float span_y() const
        {
            return count > 0 ? max_y - min_y : 0.0f;
        }
    };

    auto mean_span_for_axis = [](
        const std::unordered_map<int, AxisSpanAccumulator>& accumulators,
        DynamicBindWorldAxis axis) {
        double span_sum = 0.0;
        int group_count = 0;
        for (const auto& entry : accumulators) {
            const AxisSpanAccumulator& accumulator = entry.second;
            if (accumulator.count < 2) {
                continue;
            }
            span_sum += axis == DynamicBindWorldAxis::kX
                ? static_cast<double>(accumulator.span_x())
                : static_cast<double>(accumulator.span_y());
            group_count++;
        }
        if (group_count <= 0) {
            return std::numeric_limits<float>::quiet_NaN();
        }
        return static_cast<float>(span_sum / static_cast<double>(group_count));
    };

    DynamicBindGridAxisMapping mapping;

    std::unordered_map<int, const tie_robot_msgs::PointCoords*> point_by_global_index;
    for (const auto& world_point : planning_world_points) {
        if (world_point.idx > 0) {
            point_by_global_index[world_point.idx] = &world_point;
        }
    }

    std::unordered_map<int, AxisSpanAccumulator> rows_by_index;
    std::unordered_map<int, AxisSpanAccumulator> cols_by_index;
    for (const auto& grid_index : grid_indices) {
        if (grid_index.global_idx <= 0 ||
            grid_index.global_row < 0 ||
            grid_index.global_col < 0) {
            continue;
        }
        const auto point_it = point_by_global_index.find(grid_index.global_idx);
        if (point_it == point_by_global_index.end() || point_it->second == nullptr) {
            continue;
        }
        rows_by_index[grid_index.global_row].add(*point_it->second);
        cols_by_index[grid_index.global_col].add(*point_it->second);
    }

    mapping.row_mean_span_x_mm = mean_span_for_axis(rows_by_index, DynamicBindWorldAxis::kX);
    mapping.row_mean_span_y_mm = mean_span_for_axis(rows_by_index, DynamicBindWorldAxis::kY);
    mapping.col_mean_span_x_mm = mean_span_for_axis(cols_by_index, DynamicBindWorldAxis::kX);
    mapping.col_mean_span_y_mm = mean_span_for_axis(cols_by_index, DynamicBindWorldAxis::kY);

    const bool has_all_span_evidence =
        std::isfinite(mapping.row_mean_span_x_mm) &&
        std::isfinite(mapping.row_mean_span_y_mm) &&
        std::isfinite(mapping.col_mean_span_x_mm) &&
        std::isfinite(mapping.col_mean_span_y_mm);
    if (!has_all_span_evidence) {
        mapping.row_mean_span_x_mm = std::isfinite(mapping.row_mean_span_x_mm) ? mapping.row_mean_span_x_mm : 0.0f;
        mapping.row_mean_span_y_mm = std::isfinite(mapping.row_mean_span_y_mm) ? mapping.row_mean_span_y_mm : 0.0f;
        mapping.col_mean_span_x_mm = std::isfinite(mapping.col_mean_span_x_mm) ? mapping.col_mean_span_x_mm : 0.0f;
        mapping.col_mean_span_y_mm = std::isfinite(mapping.col_mean_span_y_mm) ? mapping.col_mean_span_y_mm : 0.0f;
        return mapping;
    }

    const float row_x_col_y_score =
        mapping.row_mean_span_x_mm + mapping.col_mean_span_y_mm;
    const float row_y_col_x_score =
        mapping.row_mean_span_y_mm + mapping.col_mean_span_x_mm;
    if (row_x_col_y_score + 1e-3f < row_y_col_x_score) {
        mapping.row_axis = DynamicBindWorldAxis::kX;
        mapping.col_axis = DynamicBindWorldAxis::kY;
        mapping.inferred_from_spans = true;
    } else if (row_y_col_x_score + 1e-3f < row_x_col_y_score) {
        mapping.row_axis = DynamicBindWorldAxis::kY;
        mapping.col_axis = DynamicBindWorldAxis::kX;
        mapping.inferred_from_spans = true;
    }
    return mapping;
}

std::vector<PseudoSlamGroupedAreaEntry> build_dynamic_bind_area_entries_from_scan_world(
    const std::vector<tie_robot_msgs::PointCoords>& planning_world_points,
    const CabinPoint& path_origin,
    float cabin_height,
    const tf2::Transform& gripper_from_base_link,
    const DynamicBindPlannerConfig& config,
    const std::vector<DynamicBindGridIndex>& grid_indices)
{
    using namespace internal;

    std::vector<PseudoSlamGroupedAreaEntry> bind_area_entries;
    if (planning_world_points.empty()) {
        return bind_area_entries;
    }
    (void)path_origin;

    std::unordered_map<int, const tie_robot_msgs::PointCoords*> world_point_by_global_index;
    for (const auto& world_point : planning_world_points) {
        if (world_point.idx > 0) {
            world_point_by_global_index[world_point.idx] = &world_point;
        }
    }

    std::unordered_map<int, std::pair<int, int>> provided_grid_by_global_index;
    std::vector<int> row_keys;
    std::vector<int> column_keys;
    for (const auto& grid_index : grid_indices) {
        if (grid_index.global_idx <= 0 ||
            grid_index.global_row < 0 ||
            grid_index.global_col < 0) {
            continue;
        }
        provided_grid_by_global_index[grid_index.global_idx] =
            {grid_index.global_row, grid_index.global_col};
        row_keys.push_back(grid_index.global_row);
        column_keys.push_back(grid_index.global_col);
    }

    const bool has_provided_grid = !provided_grid_by_global_index.empty();
    std::vector<float> column_centers;
    std::vector<float> row_centers;
    if (has_provided_grid) {
        std::sort(row_keys.begin(), row_keys.end());
        row_keys.erase(std::unique(row_keys.begin(), row_keys.end()), row_keys.end());
        std::sort(column_keys.begin(), column_keys.end());
        column_keys.erase(std::unique(column_keys.begin(), column_keys.end()), column_keys.end());
        row_keys = build_complete_non_negative_keys_to_max(row_keys);
        column_keys = build_complete_non_negative_keys_to_max(column_keys);
        row_keys = orient_grid_keys_from_world_minimum(
            std::move(row_keys),
            provided_grid_by_global_index,
            world_point_by_global_index,
            true);
        column_keys = orient_grid_keys_from_world_minimum(
            std::move(column_keys),
            provided_grid_by_global_index,
            world_point_by_global_index,
            false);
    } else {
        column_centers = cluster_world_axis_centers(
            planning_world_points,
            0,
            get_grid_axis_cluster_threshold(config, 0));
        row_centers = cluster_world_axis_centers(
            planning_world_points,
            1,
            get_grid_axis_cluster_threshold(config, 1));
        if (column_centers.empty() || row_centers.empty()) {
            return bind_area_entries;
        }
        row_keys = build_dense_keys_from_count(row_centers.size());
        column_keys = build_dense_keys_from_count(column_centers.size());
    }

    if (row_keys.empty() || column_keys.empty()) {
        return bind_area_entries;
    }

    std::unordered_map<long long, std::vector<GridPointRef>> point_refs_by_grid_cell;
    for (size_t point_index = 0; point_index < planning_world_points.size(); ++point_index) {
        const auto& world_point = planning_world_points[point_index];
        if (world_point.idx <= 0) {
            continue;
        }

        int row_index = -1;
        int column_index = -1;
        if (has_provided_grid) {
            const auto grid_index_it = provided_grid_by_global_index.find(world_point.idx);
            if (grid_index_it == provided_grid_by_global_index.end()) {
                continue;
            }
            row_index = find_dense_index_for_key(row_keys, grid_index_it->second.first);
            column_index = find_dense_index_for_key(column_keys, grid_index_it->second.second);
        } else {
            column_index = find_nearest_axis_center_index(
                world_point.World_coord[0],
                column_centers);
            row_index = find_nearest_axis_center_index(
                world_point.World_coord[1],
                row_centers);
        }
        if (row_index < 0 || column_index < 0) {
            continue;
        }

        point_refs_by_grid_cell[encode_grid_cell_key(row_index, column_index)].push_back(
            GridPointRef{row_index, column_index, &world_point});
    }
    for (auto& entry : point_refs_by_grid_cell) {
        std::sort(entry.second.begin(), entry.second.end(), [](const GridPointRef& lhs, const GridPointRef& rhs) {
            if (lhs.point == nullptr || rhs.point == nullptr) {
                return lhs.point != nullptr;
            }
            return lhs.point->idx < rhs.point->idx;
        });
    }
    relocate_duplicate_grid_refs_to_adjacent_gaps(
        point_refs_by_grid_cell,
        static_cast<int>(row_keys.size()),
        static_cast<int>(column_keys.size()));

    bool has_complete_two_by_two_grid_cells = false;
    for (int row_index = 0;
         row_index + 1 < static_cast<int>(row_keys.size()) && !has_complete_two_by_two_grid_cells;
         ++row_index) {
        for (int column_index = 0;
             column_index + 1 < static_cast<int>(column_keys.size());
             ++column_index) {
            if (has_grid_cell_refs(point_refs_by_grid_cell, row_index, column_index) &&
                has_grid_cell_refs(point_refs_by_grid_cell, row_index, column_index + 1) &&
                has_grid_cell_refs(point_refs_by_grid_cell, row_index + 1, column_index) &&
                has_grid_cell_refs(point_refs_by_grid_cell, row_index + 1, column_index + 1)) {
                has_complete_two_by_two_grid_cells = true;
                break;
            }
        }
    }

    int area_index = 1;
    std::unordered_set<long long> emitted_cell_keys;
    const int requested_group_point_count = normalize_requested_group_point_count(config);

    auto emit_group = [&](const std::vector<GridPointRef>& group_refs) {
        if (group_refs.empty()) {
            return;
        }
        const bool default_edge_pair =
            requested_group_point_count == 4 &&
            group_refs.size() == 2U &&
            are_adjacent_grid_refs(group_refs[0], group_refs[1]);
        if (static_cast<int>(group_refs.size()) != requested_group_point_count && !default_edge_pair) {
            return;
        }
        const std::pair<int, int> group_span = compute_group_grid_span(group_refs);
        if (!default_edge_pair &&
            group_span.first * group_span.second != static_cast<int>(group_refs.size())) {
            return;
        }
        const std::vector<GridPointRef> ordered_group_refs =
            order_grid_refs_by_world_y_then_x(group_refs);
        if (!is_group_reachable_from_centered_dynamic_pose(
                ordered_group_refs,
                cabin_height,
                gripper_from_base_link,
                config)) {
            return;
        }
        for (const auto& ref : ordered_group_refs) {
            const long long cell_key = encode_grid_cell_key(ref.row_index, ref.column_index);
            if (emitted_cell_keys.count(cell_key) > 0) {
                return;
            }
        }

        PseudoSlamBindGroup bind_group;
        bind_group.group_index = 1;
        bind_group.group_type = build_bind_group_type_from_refs(ordered_group_refs, config);
        for (const auto& ref : ordered_group_refs) {
            if (ref.point == nullptr) {
                continue;
            }
            bind_group.bind_points_world.push_back(*ref.point);
        }
        if (bind_group.bind_points_world.empty()) {
            return;
        }

        const DynamicBindPlanningCandidatePose candidate_pose =
            build_dynamic_bind_candidate_pose_from_world_point(
                bind_group.bind_points_world,
                gripper_from_base_link,
                config);
        PseudoSlamGroupedAreaEntry area_entry;
        area_entry.area_index = area_index++;
        area_entry.cabin_point = {candidate_pose.cabin_x, candidate_pose.cabin_y};
        area_entry.cabin_z = clamp_bind_execution_cabin_z(
            candidate_pose.cabin_z > 0.0f ? candidate_pose.cabin_z : cabin_height,
            config);
        area_entry.bind_groups.push_back(bind_group);
        bind_area_entries.push_back(area_entry);

        for (const auto& ref : ordered_group_refs) {
            emitted_cell_keys.insert(encode_grid_cell_key(ref.row_index, ref.column_index));
        }
    };

    std::vector<GridSquareCandidate> selected_square_candidates =
        requested_group_point_count != 4
            ? select_grid_group_candidates_by_requested_rectangle_tiling(
                  point_refs_by_grid_cell,
                  static_cast<int>(row_keys.size()),
                  static_cast<int>(column_keys.size()),
                  config)
            : has_provided_grid
            ? select_grid_group_candidates_by_fixed_two_by_two_tiling(
                  point_refs_by_grid_cell,
                  static_cast<int>(row_keys.size()),
                  static_cast<int>(column_keys.size()),
                  config)
            : select_grid_group_candidates_by_adjacency_matching(
                  point_refs_by_grid_cell,
                  static_cast<int>(row_keys.size()),
                  static_cast<int>(column_keys.size()),
                  config);

    sort_grid_square_candidates_by_world_x_snake(
        selected_square_candidates,
        config.snake_row_tolerance_mm);
    for (const auto& candidate : selected_square_candidates) {
        emit_group(candidate.refs);
    }

    if (!has_complete_two_by_two_grid_cells && !has_provided_grid) {
        bind_area_entries.clear();
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
