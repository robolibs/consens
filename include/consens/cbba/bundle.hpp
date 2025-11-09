#pragma once

#include "types.hpp"

#include <algorithm>
#include <vector>

namespace consens::cbba {

    /**
     * Bundle: unordered set of tasks that an agent claims
     * In CBBA, this is the 'b' vector
     */
    class Bundle {
      private:
        std::vector<TaskID> tasks_;
        size_t capacity_;

      public:
        explicit Bundle(size_t capacity = 10) : capacity_(capacity) { tasks_.reserve(capacity); }

        /**
         * Add a task to the bundle
         */
        void add(const TaskID &task_id) {
            if (!contains(task_id) && !is_full()) {
                tasks_.push_back(task_id);
            }
        }

        /**
         * Remove a task from the bundle
         */
        void remove(const TaskID &task_id) {
            auto it = std::find(tasks_.begin(), tasks_.end(), task_id);
            if (it != tasks_.end()) {
                tasks_.erase(it);
            }
        }

        /**
         * Remove all tasks from the bundle
         */
        void clear() { tasks_.clear(); }

        /**
         * Check if bundle contains a task
         */
        bool contains(const TaskID &task_id) const {
            return std::find(tasks_.begin(), tasks_.end(), task_id) != tasks_.end();
        }

        /**
         * Check if bundle is full
         */
        bool is_full() const { return tasks_.size() >= capacity_; }

        /**
         * Get number of tasks in bundle
         */
        size_t size() const { return tasks_.size(); }

        /**
         * Get capacity
         */
        size_t capacity() const { return capacity_; }

        /**
         * Get all tasks
         */
        const std::vector<TaskID> &get_tasks() const { return tasks_; }

        /**
         * Check if bundle is empty
         */
        bool empty() const { return tasks_.empty(); }
    };

    /**
     * Path: ordered sequence of tasks for execution
     * In CBBA, this is the 'p' vector (path is bundle with execution order)
     */
    class Path {
      private:
        std::vector<TaskID> tasks_;

      public:
        Path() = default;

        /**
         * Insert a task at a specific position
         */
        void insert(const TaskID &task_id, size_t position) {
            if (position > tasks_.size()) {
                position = tasks_.size();
            }
            tasks_.insert(tasks_.begin() + position, task_id);
        }

        /**
         * Remove a task from the path
         */
        void remove(const TaskID &task_id) {
            auto it = std::find(tasks_.begin(), tasks_.end(), task_id);
            if (it != tasks_.end()) {
                tasks_.erase(it);
            }
        }

        /**
         * Remove all tasks
         */
        void clear() { tasks_.clear(); }

        /**
         * Check if path contains a task
         */
        bool contains(const TaskID &task_id) const {
            return std::find(tasks_.begin(), tasks_.end(), task_id) != tasks_.end();
        }

        /**
         * Find position of a task in the path
         * Returns size() if not found
         */
        size_t find_position(const TaskID &task_id) const {
            auto it = std::find(tasks_.begin(), tasks_.end(), task_id);
            if (it != tasks_.end()) {
                return std::distance(tasks_.begin(), it);
            }
            return tasks_.size();
        }

        /**
         * Get number of tasks in path
         */
        size_t size() const { return tasks_.size(); }

        /**
         * Get all tasks in execution order
         */
        const std::vector<TaskID> &get_tasks() const { return tasks_; }

        /**
         * Check if path is empty
         */
        bool empty() const { return tasks_.empty(); }

        /**
         * Get task at specific position
         */
        const TaskID &operator[](size_t index) const { return tasks_[index]; }

        /**
         * Get first task (next to execute)
         */
        const TaskID &front() const { return tasks_.front(); }

        /**
         * Remove tasks from position onwards
         * Used when task at position is lost in consensus
         */
        void remove_from(size_t position) {
            if (position < tasks_.size()) {
                tasks_.erase(tasks_.begin() + position, tasks_.end());
            }
        }
    };

} // namespace consens::cbba
