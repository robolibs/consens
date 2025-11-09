#include "consens/cbba/scorer.hpp"

#include <cmath>
#include <limits>

namespace consens::cbba {

    TaskScorer::TaskScorer(Metric metric, double lambda) : metric_(metric), lambda_(lambda) {}

    Score TaskScorer::compute_marginal_gain(const CBBAAgent &agent, const Task &task, const Path &current_path,
                                            size_t insertion_pos, const SpatialIndex &spatial_index) const {
        // Create a temporary path with the task inserted
        Path temp_path = current_path;
        temp_path.insert(task.get_id(), insertion_pos);

        // Compute score of new path
        Score new_score = evaluate_path(agent, temp_path, spatial_index);

        // Compute score of current path
        Score current_score = evaluate_path(agent, current_path, spatial_index);

        // Marginal gain is the difference
        return new_score - current_score;
    }

    Score TaskScorer::evaluate_path(const CBBAAgent &agent, const Path &path, const SpatialIndex &spatial_index) const {
        if (metric_ == Metric::RPT) {
            return compute_rpt_score(agent, path, spatial_index);
        } else {
            return compute_tdr_score(agent, path, spatial_index);
        }
    }

    std::pair<Score, size_t> TaskScorer::find_optimal_insertion(const CBBAAgent &agent, const Task &task,
                                                                const Path &current_path,
                                                                const SpatialIndex &spatial_index) const {
        Score best_score = MIN_SCORE;
        size_t best_position = 0;

        // Try inserting at each position
        for (size_t pos = 0; pos <= current_path.size(); pos++) {
            Score marginal_gain = compute_marginal_gain(agent, task, current_path, pos, spatial_index);

            if (marginal_gain > best_score) {
                best_score = marginal_gain;
                best_position = pos;
            }
        }

        return {best_score, best_position};
    }

    double TaskScorer::compute_travel_time(const Point &from, const Point &to, double velocity) const {
        if (velocity <= 0.0) {
            return std::numeric_limits<double>::infinity();
        }

        double distance = from.distance_to(to);
        return distance / velocity;
    }

    double TaskScorer::compute_task_time(const Task &task) const {
        // For now, just use the task's duration
        // In the future, this could be more sophisticated
        return task.get_duration();
    }

    Score TaskScorer::compute_rpt_score(const CBBAAgent &agent, const Path &path,
                                        const SpatialIndex &spatial_index) const {
        if (path.empty()) {
            return 0.0;
        }

        double total_time = 0.0;
        Point current_pos = agent.get_pose().position;
        double velocity = agent.get_velocity();

        // Default velocity if not set
        if (velocity <= 0.0) {
            velocity = 2.0; // 2 m/s default
        }

        // Compute time for entire path
        for (size_t i = 0; i < path.size(); i++) {
            const TaskID &task_id = path[i];

            // Get task from spatial index
            auto task_opt = spatial_index.get_task(task_id);
            if (!task_opt) {
                continue; // Skip if task not found
            }

            const Task &task = *task_opt;

            // Travel time to task
            Point task_pos = task.get_position();
            double travel_time = compute_travel_time(current_pos, task_pos, velocity);
            total_time += travel_time;

            // Task execution time
            double task_time = compute_task_time(task);
            total_time += task_time;

            // Update current position (end of task if geometric, center otherwise)
            if (task.has_geometry()) {
                current_pos = task.get_tail(); // End at tail of geometric task
            } else {
                current_pos = task_pos;
            }
        }

        // RPT = -total_time (negative because we want to minimize time)
        // Higher score is better in CBBA, so we return negative time
        return -total_time;
    }

    Score TaskScorer::compute_tdr_score(const CBBAAgent &agent, const Path &path,
                                        const SpatialIndex &spatial_index) const {
        if (path.empty()) {
            return 0.0;
        }

        double total_reward = 0.0;
        double cumulative_time = 0.0;
        Point current_pos = agent.get_pose().position;
        double velocity = agent.get_velocity();

        // Default velocity if not set
        if (velocity <= 0.0) {
            velocity = 2.0;
        }

        // Compute time-discounted reward
        for (size_t i = 0; i < path.size(); i++) {
            const TaskID &task_id = path[i];

            // Get task from spatial index
            auto task_opt = spatial_index.get_task(task_id);
            if (!task_opt) {
                continue;
            }

            const Task &task = *task_opt;

            // Travel time to task
            Point task_pos = task.get_position();
            double travel_time = compute_travel_time(current_pos, task_pos, velocity);
            cumulative_time += travel_time;

            // Task execution time
            double task_time = compute_task_time(task);
            cumulative_time += task_time;

            // Add discounted reward: lambda^t
            double discount_factor = std::pow(lambda_, cumulative_time);
            total_reward += discount_factor;

            // Update current position
            if (task.has_geometry()) {
                current_pos = task.get_tail();
            } else {
                current_pos = task_pos;
            }
        }

        return total_reward;
    }

} // namespace consens::cbba
