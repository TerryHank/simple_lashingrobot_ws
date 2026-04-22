#include "dynamic_bind_planning_internal.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace tie_robot_process {
namespace planning {
namespace internal {

std::vector<std::vector<int>> group_candidate_indices_by_axis(
    const std::vector<PseudoSlamCandidatePoint>& candidates,
    int axis_index,
    float threshold)
{
    if (candidates.empty()) {
        return {};
    }

    std::vector<int> sorted_indices(candidates.size());
    std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
    std::sort(sorted_indices.begin(), sorted_indices.end(), [&](int lhs_index, int rhs_index) {
        const auto& lhs = candidates[lhs_index].local_point;
        const auto& rhs = candidates[rhs_index].local_point;
        if (lhs.World_coord[axis_index] != rhs.World_coord[axis_index]) {
            return lhs.World_coord[axis_index] < rhs.World_coord[axis_index];
        }
        if (lhs.World_coord[1 - axis_index] != rhs.World_coord[1 - axis_index]) {
            return lhs.World_coord[1 - axis_index] < rhs.World_coord[1 - axis_index];
        }
        return candidates[lhs_index].remaining_index < candidates[rhs_index].remaining_index;
    });

    std::vector<std::vector<int>> grouped_indices;
    std::vector<int> current_group = {sorted_indices.front()};
    double current_mean = static_cast<double>(
        candidates[sorted_indices.front()].local_point.World_coord[axis_index]);
    for (size_t sorted_pos = 1; sorted_pos < sorted_indices.size(); ++sorted_pos) {
        const int candidate_index = sorted_indices[sorted_pos];
        const double axis_value = static_cast<double>(
            candidates[candidate_index].local_point.World_coord[axis_index]);
        if (std::fabs(axis_value - current_mean) <= threshold) {
            current_group.push_back(candidate_index);
            double axis_sum = 0.0;
            for (const int grouped_index : current_group) {
                axis_sum += candidates[grouped_index].local_point.World_coord[axis_index];
            }
            current_mean = axis_sum / static_cast<double>(current_group.size());
        } else {
            grouped_indices.push_back(current_group);
            current_group = {candidate_index};
            current_mean = axis_value;
        }
    }

    if (!current_group.empty()) {
        grouped_indices.push_back(current_group);
    }
    return grouped_indices;
}

std::vector<std::tuple<int, int, float>> match_candidate_indices_between_rows(
    const std::vector<PseudoSlamCandidatePoint>& candidates,
    const std::vector<int>& upper_row,
    const std::vector<int>& lower_row,
    float column_threshold)
{
    std::vector<int> upper_sorted = upper_row;
    std::vector<int> lower_sorted = lower_row;
    auto by_local_y = [&](int lhs_index, int rhs_index) {
        const auto& lhs = candidates[lhs_index].local_point;
        const auto& rhs = candidates[rhs_index].local_point;
        if (lhs.World_coord[1] != rhs.World_coord[1]) {
            return lhs.World_coord[1] < rhs.World_coord[1];
        }
        if (lhs.World_coord[0] != rhs.World_coord[0]) {
            return lhs.World_coord[0] < rhs.World_coord[0];
        }
        return candidates[lhs_index].remaining_index < candidates[rhs_index].remaining_index;
    };
    std::sort(upper_sorted.begin(), upper_sorted.end(), by_local_y);
    std::sort(lower_sorted.begin(), lower_sorted.end(), by_local_y);

    std::vector<std::tuple<int, int, float>> matched_pairs;
    std::vector<bool> lower_used(lower_sorted.size(), false);
    for (const int upper_index : upper_sorted) {
        const float upper_y = candidates[upper_index].local_point.World_coord[1];
        int best_lower_position = -1;
        float best_gap = 0.0f;
        for (size_t lower_pos = 0; lower_pos < lower_sorted.size(); ++lower_pos) {
            if (lower_used[lower_pos]) {
                continue;
            }
            const float lower_y = candidates[lower_sorted[lower_pos]].local_point.World_coord[1];
            const float gap = std::fabs(upper_y - lower_y);
            if (gap > column_threshold) {
                continue;
            }
            if (best_lower_position < 0 || gap < best_gap) {
                best_lower_position = static_cast<int>(lower_pos);
                best_gap = gap;
            }
        }
        if (best_lower_position < 0) {
            continue;
        }

        lower_used[best_lower_position] = true;
        matched_pairs.emplace_back(
            upper_index,
            lower_sorted[best_lower_position],
            best_gap);
    }

    std::sort(matched_pairs.begin(), matched_pairs.end(), [&](const auto& lhs, const auto& rhs) {
        const auto& lhs_upper = candidates[std::get<0>(lhs)].local_point;
        const auto& lhs_lower = candidates[std::get<1>(lhs)].local_point;
        const auto& rhs_upper = candidates[std::get<0>(rhs)].local_point;
        const auto& rhs_lower = candidates[std::get<1>(rhs)].local_point;
        const float lhs_min_y = std::min(lhs_upper.World_coord[1], lhs_lower.World_coord[1]);
        const float rhs_min_y = std::min(rhs_upper.World_coord[1], rhs_lower.World_coord[1]);
        if (lhs_min_y != rhs_min_y) {
            return lhs_min_y < rhs_min_y;
        }
        if (std::get<2>(lhs) != std::get<2>(rhs)) {
            return std::get<2>(lhs) < std::get<2>(rhs);
        }
        if (lhs_upper.World_coord[0] != rhs_upper.World_coord[0]) {
            return lhs_upper.World_coord[0] < rhs_upper.World_coord[0];
        }
        return lhs_lower.World_coord[0] < rhs_lower.World_coord[0];
    });
    return matched_pairs;
}

std::vector<int> sort_matrix_candidate_indices(
    const std::vector<PseudoSlamCandidatePoint>& candidates,
    const std::vector<int>& matrix_indices)
{
    if (matrix_indices.size() != 4) {
        std::vector<int> sorted_indices = matrix_indices;
        std::sort(sorted_indices.begin(), sorted_indices.end(), [&](int lhs_index, int rhs_index) {
            const auto& lhs = candidates[lhs_index].local_point;
            const auto& rhs = candidates[rhs_index].local_point;
            if (local_origin_distance_sq(lhs) != local_origin_distance_sq(rhs)) {
                return local_origin_distance_sq(lhs) < local_origin_distance_sq(rhs);
            }
            if (lhs.World_coord[0] != rhs.World_coord[0]) {
                return lhs.World_coord[0] < rhs.World_coord[0];
            }
            if (lhs.World_coord[1] != rhs.World_coord[1]) {
                return lhs.World_coord[1] < rhs.World_coord[1];
            }
            return candidates[lhs_index].remaining_index < candidates[rhs_index].remaining_index;
        });
        return sorted_indices;
    }

    auto point1_it = std::min_element(matrix_indices.begin(), matrix_indices.end(), [&](int lhs_index, int rhs_index) {
        const auto& lhs = candidates[lhs_index].local_point;
        const auto& rhs = candidates[rhs_index].local_point;
        if (local_origin_distance_sq(lhs) != local_origin_distance_sq(rhs)) {
            return local_origin_distance_sq(lhs) < local_origin_distance_sq(rhs);
        }
        if (lhs.World_coord[0] != rhs.World_coord[0]) {
            return lhs.World_coord[0] < rhs.World_coord[0];
        }
        if (lhs.World_coord[1] != rhs.World_coord[1]) {
            return lhs.World_coord[1] < rhs.World_coord[1];
        }
        return candidates[lhs_index].remaining_index < candidates[rhs_index].remaining_index;
    });
    const int point1_index = *point1_it;
    const float point1_x = candidates[point1_index].local_point.World_coord[0];
    const float point1_y = candidates[point1_index].local_point.World_coord[1];

    std::vector<int> remaining_indices;
    for (const int candidate_index : matrix_indices) {
        if (candidate_index != point1_index) {
            remaining_indices.push_back(candidate_index);
        }
    }

    auto point2_it = std::min_element(remaining_indices.begin(), remaining_indices.end(), [&](int lhs_index, int rhs_index) {
        const auto& lhs = candidates[lhs_index].local_point;
        const auto& rhs = candidates[rhs_index].local_point;
        const auto lhs_key = std::make_tuple(
            std::fabs(lhs.World_coord[1] - point1_y),
            -std::fabs(lhs.World_coord[0] - point1_x),
            lhs.World_coord[0],
            candidates[lhs_index].remaining_index);
        const auto rhs_key = std::make_tuple(
            std::fabs(rhs.World_coord[1] - point1_y),
            -std::fabs(rhs.World_coord[0] - point1_x),
            rhs.World_coord[0],
            candidates[rhs_index].remaining_index);
        return lhs_key < rhs_key;
    });
    const int point2_index = *point2_it;
    remaining_indices.erase(std::remove(remaining_indices.begin(), remaining_indices.end(), point2_index), remaining_indices.end());

    auto point3_it = std::min_element(remaining_indices.begin(), remaining_indices.end(), [&](int lhs_index, int rhs_index) {
        const auto& lhs = candidates[lhs_index].local_point;
        const auto& rhs = candidates[rhs_index].local_point;
        const auto lhs_key = std::make_tuple(
            std::fabs(lhs.World_coord[0] - point1_x),
            -std::fabs(lhs.World_coord[1] - point1_y),
            lhs.World_coord[1],
            candidates[lhs_index].remaining_index);
        const auto rhs_key = std::make_tuple(
            std::fabs(rhs.World_coord[0] - point1_x),
            -std::fabs(rhs.World_coord[1] - point1_y),
            rhs.World_coord[1],
            candidates[rhs_index].remaining_index);
        return lhs_key < rhs_key;
    });
    const int point3_index = *point3_it;
    remaining_indices.erase(std::remove(remaining_indices.begin(), remaining_indices.end(), point3_index), remaining_indices.end());
    const int point4_index = remaining_indices.front();
    return {point1_index, point2_index, point3_index, point4_index};
}

std::vector<double> build_matrix_candidate_score(
    const std::vector<PseudoSlamCandidatePoint>& candidates,
    const std::vector<int>& sorted_matrix_indices,
    const std::vector<float>& column_gaps)
{
    std::vector<double> score_key;
    std::vector<float> x_values;
    std::vector<float> y_values;
    std::vector<float> sorted_gaps = column_gaps;
    std::sort(sorted_gaps.begin(), sorted_gaps.end());

    for (const int candidate_index : sorted_matrix_indices) {
        const auto& local_point = candidates[candidate_index].local_point;
        score_key.push_back(local_origin_distance_sq(local_point));
        score_key.push_back(local_point.World_coord[0]);
        score_key.push_back(local_point.World_coord[1]);
        score_key.push_back(static_cast<double>(candidates[candidate_index].remaining_index));
        x_values.push_back(local_point.World_coord[0]);
        y_values.push_back(local_point.World_coord[1]);
    }

    std::sort(x_values.begin(), x_values.end());
    std::sort(y_values.begin(), y_values.end());
    for (const float x_value : x_values) {
        score_key.push_back(x_value);
    }
    for (const float y_value : y_values) {
        score_key.push_back(y_value);
    }
    for (const float gap : sorted_gaps) {
        score_key.push_back(gap);
    }
    return score_key;
}

std::vector<int> select_nearest_origin_matrix_candidate_indices(
    const std::vector<PseudoSlamCandidatePoint>& candidates,
    const DynamicBindPlannerConfig& config)
{
    if (candidates.size() < 4) {
        return {};
    }

    const std::vector<std::vector<int>> rows = group_candidate_indices_by_axis(
        candidates,
        0,
        config.matrix_row_threshold_mm);
    if (rows.size() < 2) {
        return {};
    }

    std::vector<int> best_indices;
    std::vector<double> best_score;
    for (size_t upper_index = 0; upper_index + 1 < rows.size(); ++upper_index) {
        const auto& upper_row = rows[upper_index];
        if (upper_row.size() < 2) {
            continue;
        }

        for (size_t lower_index = upper_index + 1; lower_index < rows.size(); ++lower_index) {
            const auto& lower_row = rows[lower_index];
            if (lower_row.size() < 2) {
                continue;
            }

            const auto matched_pairs = match_candidate_indices_between_rows(
                candidates,
                upper_row,
                lower_row,
                config.matrix_column_threshold_mm);
            if (matched_pairs.size() < 2) {
                continue;
            }

            for (size_t first_pair_index = 0; first_pair_index + 1 < matched_pairs.size(); ++first_pair_index) {
                for (size_t second_pair_index = first_pair_index + 1; second_pair_index < matched_pairs.size(); ++second_pair_index) {
                    const auto& first_pair = matched_pairs[first_pair_index];
                    const auto& second_pair = matched_pairs[second_pair_index];
                    std::vector<int> selected_indices = {
                        std::get<0>(first_pair),
                        std::get<1>(first_pair),
                        std::get<0>(second_pair),
                        std::get<1>(second_pair),
                    };
                    const std::vector<int> sorted_indices = sort_matrix_candidate_indices(candidates, selected_indices);
                    const std::vector<double> score = build_matrix_candidate_score(
                        candidates,
                        sorted_indices,
                        {std::get<2>(first_pair), std::get<2>(second_pair)});
                    if (best_indices.empty() || score < best_score) {
                        best_indices = sorted_indices;
                        best_score = score;
                    }
                }
            }
        }
    }

    return best_indices;
}

}  // namespace internal
}  // namespace planning
}  // namespace tie_robot_process
