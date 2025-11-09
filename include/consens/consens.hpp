#pragma once

#include "task.hpp"
#include "types.hpp"

#include <map>
#include <memory>
#include <vector>

namespace consens {

    /**
     * Main Consens class - one instance per agent
     * Completely independent of any simulator
     */
    class Consens {
      public:
        /**
         * Configuration for this agent's consens instance
         */
        struct Config {
            AgentID agent_id;
            size_t max_bundle_size = 10;
            float spatial_query_radius = 100.0f;
            bool enable_logging = true;

            // Communication callbacks
            SendCallback send_message;
            ReceiveCallback receive_messages;
        };

        /**
         * Statistics about allocation
         */
        struct Statistics {
            size_t iteration_count;
            size_t bundle_size;
            size_t total_tasks;
            double total_path_score;
            bool converged;
        };

        /**
         * Constructor
         */
        explicit Consens(const Config &config);

        /**
         * Destructor
         */
        ~Consens();

        // ========== State Updates (called by simulator) ==========

        /**
         * Update agent's current pose
         */
        void update_pose(double x, double y, double heading);
        void update_pose(const Point &position, double heading);
        void update_pose(const Pose &pose);

        /**
         * Update agent's velocity (m/s)
         */
        void update_velocity(double velocity);

        /**
         * Add a point task to the world
         */
        void add_task(const TaskID &id, const Point &position, double duration);

        /**
         * Add a geometric task (e.g., row/swath) to the world
         */
        void add_task(const TaskID &id, const Point &head, const Point &tail, double duration);

        /**
         * Add a task object directly
         */
        void add_task(const Task &task);

        /**
         * Remove a task (completed or canceled)
         */
        void remove_task(const TaskID &id);

        /**
         * Mark task as completed
         */
        void mark_task_completed(const TaskID &id);

        /**
         * Update list of neighboring agents (for communication)
         */
        void update_neighbors(const std::vector<AgentID> &neighbor_ids);

        // ========== Main Execution ==========

        /**
         * Run one iteration of the allocation algorithm
         * Called by simulator every frame
         */
        void tick(float dt);

        // ========== Query Results ==========

        /**
         * Get current bundle (unordered list of tasks this agent claims)
         */
        std::vector<TaskID> get_bundle() const;

        /**
         * Get current path (ordered list of tasks to execute)
         */
        std::vector<TaskID> get_path() const;

        /**
         * Get next task to execute (first in path)
         */
        std::optional<TaskID> get_next_task() const;

        /**
         * Get task details by ID
         */
        std::optional<Task> get_task(const TaskID &id) const;

        /**
         * Get all tasks
         */
        std::vector<Task> get_all_tasks() const;

        /**
         * Check if algorithm has converged
         */
        bool has_converged() const;

        /**
         * Get allocation statistics
         */
        Statistics get_statistics() const;

        /**
         * Get agent ID
         */
        const AgentID &get_agent_id() const;

        /**
         * Get current pose
         */
        Pose get_pose() const;

        /**
         * Get current velocity
         */
        double get_velocity() const;

        /**
         * Reset algorithm state (clears bundle, bids, etc.)
         */
        void reset();

      private:
        // Forward declaration of implementation
        class Impl;
        std::unique_ptr<Impl> impl_;
    };

} // namespace consens
