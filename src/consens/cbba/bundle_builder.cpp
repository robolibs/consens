#include "consens/cbba/bundle_builder.hpp"

#include <algorithm>

namespace consens::cbba {

    BundleBuilder::BundleBuilder(SpatialIndex *spatial_index, Metric metric, float query_radius, BundleMode mode)
        : scorer_(metric), spatial_index_(spatial_index), query_radius_(query_radius), mode_(mode) {}

    void BundleBuilder::build_bundle(CBBAAgent &agent, const std::vector<TaskID> &available_tasks) {
        if (mode_ == BundleMode::ADD) {
            add_one_task(agent, available_tasks);
        } else {
            fill_bundle(agent, available_tasks);
        }
    }

    std::vector<TaskID> BundleBuilder::get_candidates(const CBBAAgent &agent,
                                                      const std::vector<TaskID> &available_tasks) const {
        std::vector<TaskID> candidates;

        // Get agent's current position
        Point agent_pos = agent.get_pose().position;

        // Query spatial index for nearby tasks
        std::vector<TaskID> nearby_tasks = spatial_index_->query_radius(agent_pos, query_radius_);

        // Filter to only include available tasks
        for (const auto &task_id : nearby_tasks) {
            if (std::find(available_tasks.begin(), available_tasks.end(), task_id) != available_tasks.end()) {
                candidates.push_back(task_id);
            }
        }

        return candidates;
    }

    std::tuple<TaskID, Score, size_t> BundleBuilder::find_best_task(const CBBAAgent &agent,
                                                                    const std::vector<TaskID> &candidates) {
        TaskID best_task_id = "";
        Score best_score = MIN_SCORE;
        size_t best_position = 0;

        // Try each candidate task
        for (const auto &task_id : candidates) {
            // Skip if already in bundle
            if (agent.get_bundle().contains(task_id)) {
                continue;
            }

            // Get task from spatial index
            auto task_opt = spatial_index_->get_task(task_id);
            if (!task_opt) {
                continue;
            }

            const Task &task = *task_opt;

            // Find optimal insertion position and score
            auto [score, position] = scorer_.find_optimal_insertion(agent, task, agent.get_path(), *spatial_index_);

            // Check if this is better
            if (score > best_score) {
                best_score = score;
                best_task_id = task_id;
                best_position = position;
            }
        }

        return {best_task_id, best_score, best_position};
    }

    bool BundleBuilder::should_bid(const CBBAAgent &agent, const TaskID &task_id, Score my_bid) const {
        // Get current winning bid for this task
        Bid winning_bid = agent.get_winning_bid(task_id);

        // If no winner yet, we should bid
        if (!winning_bid.is_valid()) {
            return true;
        }

        // Create our bid
        Bid our_bid(agent.get_id(), my_bid, agent.get_timestamp(agent.get_id()));

        // Bid if ours is better
        return our_bid > winning_bid;
    }

    bool BundleBuilder::add_one_task(CBBAAgent &agent, const std::vector<TaskID> &available_tasks) {
        // Check if bundle is full
        if (agent.get_bundle().is_full()) {
            return false;
        }

        // Get candidate tasks using spatial filtering
        auto candidates = get_candidates(agent, available_tasks);

        if (candidates.empty()) {
            return false;
        }

        // Find best task to add
        auto [best_task_id, best_score, best_position] = find_best_task(agent, candidates);

        // Check if we found a valid task
        if (best_task_id.empty()) {
            return false;
        }

        // Check if we should bid on this task
        if (!should_bid(agent, best_task_id, best_score)) {
            return false;
        }

        // Add task to bundle
        agent.add_to_bundle(best_task_id, best_score, best_position);

        return true;
    }

    size_t BundleBuilder::fill_bundle(CBBAAgent &agent, const std::vector<TaskID> &available_tasks) {
        size_t added_count = 0;

        // Keep adding tasks until bundle is full or no more tasks can be added
        while (!agent.get_bundle().is_full()) {
            bool added = add_one_task(agent, available_tasks);
            if (!added) {
                break; // No more tasks to add
            }
            added_count++;
        }

        return added_count;
    }

} // namespace consens::cbba
