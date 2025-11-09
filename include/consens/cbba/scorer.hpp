#pragma once

#include "../task.hpp"
#include "bundle.hpp"
#include "cbba_agent.hpp"
#include "spatial_index.hpp"
#include "types.hpp"

namespace consens::cbba {

    /**
     * Task scorer for computing utilities in CBBA
     * Implements various scoring metrics (RPT, TDR)
     */
    class TaskScorer {
      private:
        Metric metric_;
        double lambda_; // Discount factor for TDR metric

      public:
        /**
         * Constructor
         * @param metric Scoring metric to use (default: RPT)
         * @param lambda Discount factor for TDR (default: 0.95)
         */
        explicit TaskScorer(Metric metric = Metric::RPT, double lambda = 0.95);

        /**
         * Compute marginal gain of adding a task to the path
         * This is the key scoring function for CBBA
         *
         * @param agent Agent state
         * @param task Task to add
         * @param current_path Current path
         * @param insertion_pos Position to insert task in path
         * @param spatial_index Spatial index for looking up tasks
         * @return Marginal utility (score) of adding this task
         */
        Score compute_marginal_gain(const CBBAAgent &agent, const Task &task, const Path &current_path,
                                    size_t insertion_pos, const SpatialIndex &spatial_index) const;

        /**
         * Evaluate the score of an entire path
         *
         * @param agent Agent state
         * @param path Path to evaluate
         * @param spatial_index Spatial index for looking up tasks
         * @return Total score of the path
         */
        Score evaluate_path(const CBBAAgent &agent, const Path &path, const SpatialIndex &spatial_index) const;

        /**
         * Find optimal insertion position for a task in the path
         * Tries all positions and returns the one with best marginal gain
         *
         * @param agent Agent state
         * @param task Task to insert
         * @param current_path Current path
         * @param spatial_index Spatial index for looking up tasks
         * @return Pair of (best_score, best_position)
         */
        std::pair<Score, size_t> find_optimal_insertion(const CBBAAgent &agent, const Task &task,
                                                        const Path &current_path,
                                                        const SpatialIndex &spatial_index) const;

        /**
         * Get current metric
         */
        Metric get_metric() const { return metric_; }

        /**
         * Set metric
         */
        void set_metric(Metric metric) { metric_ = metric; }

      private:
        /**
         * Compute travel time between two points
         * @param from Start point
         * @param to End point
         * @param velocity Agent velocity (m/s)
         * @return Time in seconds
         */
        double compute_travel_time(const Point &from, const Point &to, double velocity) const;

        /**
         * Compute time to complete a task
         * @param task Task to complete
         * @return Time in seconds (just the task duration for now)
         */
        double compute_task_time(const Task &task) const;

        /**
         * Compute total time for a path using RPT metric
         * RPT = Reward Per Time = -total_time (we want to minimize time)
         *
         * @param agent Agent state
         * @param path Path to evaluate
         * @param spatial_index Spatial index
         * @return Negative total time (higher is better)
         */
        Score compute_rpt_score(const CBBAAgent &agent, const Path &path, const SpatialIndex &spatial_index) const;

        /**
         * Compute time-discounted reward for a path using TDR metric
         * TDR = sum of lambda^t_i for each task
         *
         * @param agent Agent state
         * @param path Path to evaluate
         * @param spatial_index Spatial index
         * @return Time-discounted reward (higher is better)
         */
        Score compute_tdr_score(const CBBAAgent &agent, const Path &path, const SpatialIndex &spatial_index) const;
    };

} // namespace consens::cbba
