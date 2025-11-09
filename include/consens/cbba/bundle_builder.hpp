#pragma once

#include "cbba_agent.hpp"
#include "scorer.hpp"
#include "spatial_index.hpp"
#include "types.hpp"

#include <vector>

namespace consens::cbba {

    /**
     * Bundle builder implements the bundle construction phase of CBBA
     * Uses spatial filtering and greedy task selection
     */
    class BundleBuilder {
      private:
        TaskScorer scorer_;
        SpatialIndex *spatial_index_;
        float query_radius_;
        BundleMode mode_;

      public:
        /**
         * Constructor
         * @param spatial_index Pointer to spatial index (not owned)
         * @param metric Scoring metric (default: RPT)
         * @param query_radius Radius for spatial queries (default: 100m)
         * @param mode Bundle building mode (default: ADD)
         */
        BundleBuilder(SpatialIndex *spatial_index, Metric metric = Metric::RPT, float query_radius = 100.0f,
                      BundleMode mode = BundleMode::ADD);

        /**
         * Build bundle for an agent
         *
         * In ADD mode: Adds one best task per call
         * In FULLBUNDLE mode: Fills bundle to capacity
         *
         * @param agent Agent to build bundle for
         * @param available_tasks List of task IDs that are unassigned or can be bid on
         */
        void build_bundle(CBBAAgent &agent, const std::vector<TaskID> &available_tasks);

        /**
         * Set spatial query radius
         */
        void set_query_radius(float radius) { query_radius_ = radius; }

        /**
         * Get current query radius
         */
        float get_query_radius() const { return query_radius_; }

        /**
         * Set bundle building mode
         */
        void set_mode(BundleMode mode) { mode_ = mode; }

        /**
         * Get current mode
         */
        BundleMode get_mode() const { return mode_; }

        /**
         * Set scoring metric
         */
        void set_metric(Metric metric) { scorer_.set_metric(metric); }

        /**
         * Get current metric
         */
        Metric get_metric() const { return scorer_.get_metric(); }

      private:
        /**
         * Get candidate tasks using spatial filtering
         * Queries spatial index for tasks near agent's position
         *
         * @param agent Agent state
         * @param available_tasks List of available task IDs
         * @return Filtered list of candidate task IDs
         */
        std::vector<TaskID> get_candidates(const CBBAAgent &agent, const std::vector<TaskID> &available_tasks) const;

        /**
         * Find best task to add to bundle
         * Computes marginal gain for each candidate and selects best
         *
         * @param agent Agent state
         * @param candidates List of candidate task IDs
         * @return Tuple of (best_task_id, best_score, best_position), or empty task_id if none found
         */
        std::tuple<TaskID, Score, size_t> find_best_task(const CBBAAgent &agent, const std::vector<TaskID> &candidates);

        /**
         * Check if agent should bid on a task
         * Returns true if agent's bid is better than current winning bid
         *
         * @param agent Agent state
         * @param task_id Task to check
         * @param my_bid Agent's computed bid
         * @return True if agent should bid
         */
        bool should_bid(const CBBAAgent &agent, const TaskID &task_id, Score my_bid) const;

        /**
         * Add one task to bundle (ADD mode)
         * @return True if a task was added
         */
        bool add_one_task(CBBAAgent &agent, const std::vector<TaskID> &available_tasks);

        /**
         * Fill bundle to capacity (FULLBUNDLE mode)
         * @return Number of tasks added
         */
        size_t fill_bundle(CBBAAgent &agent, const std::vector<TaskID> &available_tasks);
    };

} // namespace consens::cbba
