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

bool are_adjacent_grid_refs(const GridPointRef& left, const GridPointRef& right)
{
    const int row_delta = std::abs(left.row_index - right.row_index);
    const int column_delta = std::abs(left.column_index - right.column_index);
    return (row_delta == 0 && column_delta == 1) ||
           (row_delta == 1 && column_delta == 0);
}

std::pair<int, int> find_best_edge_pair_indices(const std::vector<GridPointRef>& group_refs)
{
    std::pair<int, int> best_pair{-1, -1};
    int best_score = -1;
    for (size_t left_index = 0; left_index < group_refs.size(); ++left_index) {
        for (size_t right_index = left_index + 1; right_index < group_refs.size(); ++right_index) {
            const GridPointRef& left_ref = group_refs[left_index];
            const GridPointRef& right_ref = group_refs[right_index];
            if (!are_adjacent_grid_refs(left_ref, right_ref)) {
                continue;
            }
            int score = 1;
            if (left_ref.row_index == right_ref.row_index) {
                score += 10;
            }
            if (best_pair.first < 0 || score > best_score) {
                best_pair = {static_cast<int>(left_index), static_cast<int>(right_index)};
                best_score = score;
            }
        }
    }
    return best_pair;
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
        static_cast<double>(config.template_center_x_mm));
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

int build_grid_square_traversal_order(int row_index, int column_index, int row_band_count)
{
    const int column_band_index = column_index / 2;
    const int row_band_index = row_index / 2;
    const bool moving_up = (column_band_index % 2) == 0;
    const int row_order_index =
        moving_up ? row_band_index : (row_band_count - 1 - row_band_index);
    return column_band_index * std::max(row_band_count, 1) + row_order_index;
}

int find_dense_index_for_key(const std::vector<int>& sorted_keys, int key)
{
    const auto key_it = std::lower_bound(sorted_keys.begin(), sorted_keys.end(), key);
    if (key_it == sorted_keys.end() || *key_it != key) {
        return -1;
    }
    return static_cast<int>(std::distance(sorted_keys.begin(), key_it));
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
        static_cast<double>(config.template_center_x_mm));
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

int build_grid_group_traversal_order(int row_index, int column_index, int row_band_count)
{
    const int column_band_index = column_index / 2;
    const int row_band_index = row_index / 2;
    const bool moving_up = (column_band_index % 2) == 0;
    const int row_order_index =
        moving_up ? row_band_index : (row_band_count - 1 - row_band_index);
    return column_band_index * std::max(row_band_count, 1) + row_order_index;
}

struct GridTilingStateKey
{
    int position = 0;
    std::array<unsigned long long, 6> used_masks{{0ULL, 0ULL, 0ULL, 0ULL, 0ULL, 0ULL}};

    bool operator==(const GridTilingStateKey& other) const
    {
        return position == other.position && used_masks == other.used_masks;
    }
};

struct GridTilingStateKeyHash
{
    size_t operator()(const GridTilingStateKey& key) const
    {
        size_t seed = static_cast<size_t>(key.position);
        for (const unsigned long long mask : key.used_masks) {
            seed ^= static_cast<size_t>(mask + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2));
        }
        return seed;
    }
};

struct GridTilingScore
{
    int covered_cell_count = -1;
    int square_count = -1;
    double geometry_score = std::numeric_limits<double>::max();
    double traversal_score = std::numeric_limits<double>::max();
};

bool is_better_grid_tiling_score(const GridTilingScore& candidate, const GridTilingScore& current)
{
    if (candidate.covered_cell_count != current.covered_cell_count) {
        return candidate.covered_cell_count > current.covered_cell_count;
    }
    if (candidate.square_count != current.square_count) {
        return candidate.square_count > current.square_count;
    }
    if (std::fabs(candidate.geometry_score - current.geometry_score) > 1e-6) {
        return candidate.geometry_score < current.geometry_score;
    }
    return candidate.traversal_score < current.traversal_score;
}

GridTilingScore add_grid_tile_score(
    const GridTilingScore& suffix_score,
    int covered_cell_count,
    int square_count,
    double geometry_score,
    double traversal_score)
{
    if (suffix_score.covered_cell_count < 0) {
        return suffix_score;
    }
    return GridTilingScore{
        suffix_score.covered_cell_count + covered_cell_count,
        suffix_score.square_count + square_count,
        suffix_score.geometry_score + geometry_score,
        suffix_score.traversal_score + traversal_score,
    };
}

std::vector<GridSquareCandidate> select_grid_group_candidates_by_coverage(
    const std::unordered_map<long long, std::vector<GridPointRef>>& point_refs_by_grid_cell,
    int row_count,
    int column_count,
    int row_band_count,
    const DynamicBindPlannerConfig& config)
{
    std::vector<GridSquareCandidate> selected_candidates;
    constexpr int kMaskBlockCount = 6;
    const int cell_count = row_count * column_count;
    if (row_count <= 0 || column_count <= 0 || cell_count > kMaskBlockCount * 64) {
        return selected_candidates;
    }

    auto find_cell_refs = [&](int row_index, int column_index) -> const std::vector<GridPointRef>* {
        const auto cell_it = point_refs_by_grid_cell.find(encode_grid_cell_key(row_index, column_index));
        if (cell_it == point_refs_by_grid_cell.end() || cell_it->second.empty()) {
            return nullptr;
        }
        return &cell_it->second;
    };

    auto cell_position = [&](int row_index, int column_index) {
        return row_index * column_count + column_index;
    };

    auto is_mask_bit_set = [](const std::array<unsigned long long, kMaskBlockCount>& masks, int position) {
        return (masks[static_cast<size_t>(position / 64)] &
                (1ULL << static_cast<unsigned int>(position % 64))) != 0ULL;
    };

    auto set_mask_bit = [](std::array<unsigned long long, kMaskBlockCount>& masks, int position) {
        masks[static_cast<size_t>(position / 64)] |=
            1ULL << static_cast<unsigned int>(position % 64);
    };

    auto is_cell_available = [&](int row_index, int column_index, const std::array<unsigned long long, kMaskBlockCount>& used_masks) -> bool {
        if (row_index < 0 || column_index < 0 || row_index >= row_count || column_index >= column_count) {
            return false;
        }
        const int position = cell_position(row_index, column_index);
        return !is_mask_bit_set(used_masks, position) && find_cell_refs(row_index, column_index) != nullptr;
    };

    enum class ChoiceKind
    {
        Skip,
        Square,
        HorizontalPair,
        VerticalPair,
    };

    std::unordered_map<GridTilingStateKey, GridTilingScore, GridTilingStateKeyHash> memo;
    std::unordered_map<GridTilingStateKey, ChoiceKind, GridTilingStateKeyHash> choice_by_state;

    auto find_next_open_position = [&](int start_position, const std::array<unsigned long long, kMaskBlockCount>& used_masks) {
        for (int position = start_position; position < cell_count; ++position) {
            const int row_index = position / column_count;
            const int column_index = position % column_count;
            if (find_cell_refs(row_index, column_index) != nullptr &&
                !is_mask_bit_set(used_masks, position)) {
                return position;
            }
        }
        return cell_count;
    };

    std::function<GridTilingScore(int, const std::array<unsigned long long, kMaskBlockCount>&)> solve =
        [&](int start_position, const std::array<unsigned long long, kMaskBlockCount>& used_masks) -> GridTilingScore {
            const int position = find_next_open_position(start_position, used_masks);
            if (position >= cell_count) {
                return GridTilingScore{0, 0, 0.0, 0.0};
            }

            const GridTilingStateKey state_key{position, used_masks};
            const auto memo_it = memo.find(state_key);
            if (memo_it != memo.end()) {
                return memo_it->second;
            }

            const int row_index = position / column_count;
            const int column_index = position % column_count;
            const auto* current_cell_refs = find_cell_refs(row_index, column_index);

            auto skipped_masks = used_masks;
            set_mask_bit(skipped_masks, position);
            GridTilingScore best_score = solve(position + 1, skipped_masks);
            ChoiceKind best_choice = ChoiceKind::Skip;
            const double traversal_score =
                static_cast<double>(build_grid_group_traversal_order(row_index, column_index, row_band_count)) * 1000000.0 +
                static_cast<double>(column_index) * 1000.0 +
                static_cast<double>(row_index);

            if (is_cell_available(row_index, column_index + 1, used_masks) &&
                is_cell_available(row_index + 1, column_index, used_masks) &&
                is_cell_available(row_index + 1, column_index + 1, used_masks)) {
                const auto* top_right_refs = find_cell_refs(row_index, column_index + 1);
                const auto* bottom_left_refs = find_cell_refs(row_index + 1, column_index);
                const auto* bottom_right_refs = find_cell_refs(row_index + 1, column_index + 1);
                double geometry_score = 0.0;
                const std::vector<GridPointRef> square_refs = select_best_square_refs(
                    *current_cell_refs,
                    *top_right_refs,
                    *bottom_left_refs,
                    *bottom_right_refs,
                    config,
                    geometry_score);
                if (square_refs.size() == 4U) {
                    auto square_masks = used_masks;
                    set_mask_bit(square_masks, position);
                    set_mask_bit(square_masks, cell_position(row_index, column_index + 1));
                    set_mask_bit(square_masks, cell_position(row_index + 1, column_index));
                    set_mask_bit(square_masks, cell_position(row_index + 1, column_index + 1));
                    const GridTilingScore candidate_score = add_grid_tile_score(
                        solve(position + 1, square_masks),
                        4,
                        1,
                        geometry_score,
                        traversal_score);
                    if (is_better_grid_tiling_score(candidate_score, best_score)) {
                        best_score = candidate_score;
                        best_choice = ChoiceKind::Square;
                    }
                }
            }

            if (is_cell_available(row_index, column_index + 1, used_masks)) {
                const auto* right_refs = find_cell_refs(row_index, column_index + 1);
                const GridPointRef left_ref = current_cell_refs->front();
                const GridPointRef right_ref = right_refs->front();
                auto pair_masks = used_masks;
                set_mask_bit(pair_masks, position);
                set_mask_bit(pair_masks, cell_position(row_index, column_index + 1));
                const GridTilingScore candidate_score = add_grid_tile_score(
                    solve(position + 1, pair_masks),
                    2,
                    0,
                    pair_candidate_geometry_score(left_ref, right_ref, config),
                    traversal_score + 100.0);
                if (is_better_grid_tiling_score(candidate_score, best_score)) {
                    best_score = candidate_score;
                    best_choice = ChoiceKind::HorizontalPair;
                }
            }

            if (is_cell_available(row_index + 1, column_index, used_masks)) {
                const auto* bottom_refs = find_cell_refs(row_index + 1, column_index);
                const GridPointRef top_ref = current_cell_refs->front();
                const GridPointRef bottom_ref = bottom_refs->front();
                auto pair_masks = used_masks;
                set_mask_bit(pair_masks, position);
                set_mask_bit(pair_masks, cell_position(row_index + 1, column_index));
                const GridTilingScore candidate_score = add_grid_tile_score(
                    solve(position + 1, pair_masks),
                    2,
                    0,
                    pair_candidate_geometry_score(top_ref, bottom_ref, config),
                    traversal_score + 200.0);
                if (is_better_grid_tiling_score(candidate_score, best_score)) {
                    best_score = candidate_score;
                    best_choice = ChoiceKind::VerticalPair;
                }
            }

            memo[state_key] = best_score;
            choice_by_state[state_key] = best_choice;
            return best_score;
        };

    std::array<unsigned long long, kMaskBlockCount> used_masks{{0ULL, 0ULL, 0ULL, 0ULL, 0ULL, 0ULL}};
    solve(0, used_masks);

    int position = 0;
    while (position < cell_count) {
        position = find_next_open_position(position, used_masks);
        if (position >= cell_count) {
            break;
        }

        const int row_index = position / column_count;
        const int column_index = position % column_count;
        const auto* current_cell_refs = find_cell_refs(row_index, column_index);
        const GridTilingStateKey state_key{position, used_masks};
        const auto choice_it = choice_by_state.find(state_key);
        const ChoiceKind choice = choice_it != choice_by_state.end() ? choice_it->second : ChoiceKind::Skip;
        if (choice == ChoiceKind::Square) {
            const auto* top_right_refs = find_cell_refs(row_index, column_index + 1);
            const auto* bottom_left_refs = find_cell_refs(row_index + 1, column_index);
            const auto* bottom_right_refs = find_cell_refs(row_index + 1, column_index + 1);
            double geometry_score = 0.0;
            std::vector<GridPointRef> square_refs = select_best_square_refs(
                *current_cell_refs,
                *top_right_refs,
                *bottom_left_refs,
                *bottom_right_refs,
                config,
                geometry_score);
            if (((column_index / 2) % 2) != 0) {
                square_refs = {
                    square_refs[2],
                    square_refs[3],
                    square_refs[0],
                    square_refs[1],
                };
            }
            selected_candidates.push_back(GridSquareCandidate{
                row_index,
                column_index,
                build_grid_group_traversal_order(row_index, column_index, row_band_count),
                geometry_score,
                square_refs,
            });
            set_mask_bit(used_masks, position);
            set_mask_bit(used_masks, cell_position(row_index, column_index + 1));
            set_mask_bit(used_masks, cell_position(row_index + 1, column_index));
            set_mask_bit(used_masks, cell_position(row_index + 1, column_index + 1));
            position++;
            continue;
        }
        if (choice == ChoiceKind::HorizontalPair) {
            const auto* right_refs = find_cell_refs(row_index, column_index + 1);
            selected_candidates.push_back(GridSquareCandidate{
                row_index,
                column_index,
                build_grid_group_traversal_order(row_index, column_index, row_band_count),
                pair_candidate_geometry_score(current_cell_refs->front(), right_refs->front(), config),
                {current_cell_refs->front(), right_refs->front()},
            });
            set_mask_bit(used_masks, position);
            set_mask_bit(used_masks, cell_position(row_index, column_index + 1));
            position++;
            continue;
        }
        if (choice == ChoiceKind::VerticalPair) {
            const auto* bottom_refs = find_cell_refs(row_index + 1, column_index);
            selected_candidates.push_back(GridSquareCandidate{
                row_index,
                column_index,
                build_grid_group_traversal_order(row_index, column_index, row_band_count),
                pair_candidate_geometry_score(current_cell_refs->front(), bottom_refs->front(), config),
                {current_cell_refs->front(), bottom_refs->front()},
            });
            set_mask_bit(used_masks, position);
            set_mask_bit(used_masks, cell_position(row_index + 1, column_index));
            position++;
            continue;
        }

        set_mask_bit(used_masks, position);
        position++;
    }

    return selected_candidates;
}

}  // namespace

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

    constexpr int kGroupStride = 2;
    const int row_band_count =
        static_cast<int>((row_keys.size() + static_cast<size_t>(kGroupStride - 1)) /
                         static_cast<size_t>(kGroupStride));
    int area_index = 1;
    bool has_full_matrix_group = false;
    std::unordered_set<long long> emitted_cell_keys;

    auto emit_group = [&](const std::vector<GridPointRef>& group_refs) {
        if (group_refs.empty()) {
            return;
        }
        if (group_refs.size() != 2U && group_refs.size() != 4U) {
            return;
        }
        if (group_refs.size() == 2U && !are_adjacent_grid_refs(group_refs[0], group_refs[1])) {
            return;
        }
        for (const auto& ref : group_refs) {
            const long long cell_key = encode_grid_cell_key(ref.row_index, ref.column_index);
            if (emitted_cell_keys.count(cell_key) > 0) {
                return;
            }
        }

        PseudoSlamBindGroup bind_group;
        bind_group.group_index = 1;
        bind_group.group_type =
            group_refs.size() == 4U ? "matrix_2x2" : "matrix_2x2_edge_pair";
        for (const auto& ref : group_refs) {
            if (ref.point == nullptr) {
                continue;
            }
            bind_group.bind_points_world.push_back(*ref.point);
        }
        if (bind_group.bind_points_world.empty()) {
            return;
        }

        has_full_matrix_group =
            has_full_matrix_group || bind_group.bind_points_world.size() == 4U;
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

        for (const auto& ref : group_refs) {
            emitted_cell_keys.insert(encode_grid_cell_key(ref.row_index, ref.column_index));
        }
    };

    std::vector<GridSquareCandidate> selected_square_candidates =
        select_grid_group_candidates_by_coverage(
            point_refs_by_grid_cell,
            static_cast<int>(row_keys.size()),
            static_cast<int>(column_keys.size()),
            row_band_count,
            config);

    std::sort(selected_square_candidates.begin(), selected_square_candidates.end(), [](const GridSquareCandidate& lhs, const GridSquareCandidate& rhs) {
        if (lhs.traversal_order != rhs.traversal_order) {
            return lhs.traversal_order < rhs.traversal_order;
        }
        if (lhs.column_index != rhs.column_index) {
            return lhs.column_index < rhs.column_index;
        }
        return lhs.row_index < rhs.row_index;
    });
    for (const auto& candidate : selected_square_candidates) {
        emit_group(candidate.refs);
    }

    if (!has_full_matrix_group) {
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
