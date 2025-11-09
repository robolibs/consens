#pragma once

#include "task.hpp"
#include "types.hpp"

#include <optional>
#include <vector>

namespace consens {

    /**
     * Abstract base class for consensus algorithms
     * This interface allows different algorithms (CBBA, sequential greedy, etc.)
     * to be plugged into the Consens framework
     */
    class Algorithm {
      public:
        virtual ~Algorithm() = default;

        /**
         * Update agent's current pose
         */
        virtual void update_pose(const Pose &pose) = 0;

        /**
         * Update agent's velocity
         */
        virtual void update_velocity(double velocity) = 0;

        /**
         * Add a task to the world
         */
        virtual void add_task(const Task &task) = 0;

        /**
         * Remove a task (completed or canceled)
         */
        virtual void remove_task(const TaskID &id) = 0;

        /**
         * Mark task as completed
         */
        virtual void mark_task_completed(const TaskID &id) = 0;

        /**
         * Run one iteration of the algorithm
         * This is where the main algorithm logic happens
         *
         * @param dt Delta time since last tick (seconds)
         */
        virtual void tick(float dt) = 0;

        /**
         * Get current bundle (tasks claimed by this agent)
         */
        virtual std::vector<TaskID> get_bundle() const = 0;

        /**
         * Get current path (ordered execution sequence)
         */
        virtual std::vector<TaskID> get_path() const = 0;

        /**
         * Get next task to execute
         */
        virtual std::optional<TaskID> get_next_task() const = 0;

        /**
         * Get task details by ID
         */
        virtual std::optional<Task> get_task(const TaskID &id) const = 0;

        /**
         * Get all tasks
         */
        virtual std::vector<Task> get_all_tasks() const = 0;

        /**
         * Check if algorithm has converged
         */
        virtual bool has_converged() const = 0;

        /**
         * Reset algorithm state
         */
        virtual void reset() = 0;

        /**
         * Get total score/cost of current allocation
         */
        virtual double get_total_score() const = 0;
    };

} // namespace consens
