#pragma once

#include "types.hpp"

namespace consens {

    /**
     * Task representation
     * Can represent a simple point task or a geometric task (like a row/swath)
     */
    class Task {
      public:
        /**
         * Default constructor
         */
        Task() = default;

        /**
         * Create a point task (single location)
         */
        Task(const TaskID &id, const Point &position, double duration);

        /**
         * Create a geometric task (e.g., row/swath with start and end)
         */
        Task(const TaskID &id, const Point &head, const Point &tail, double duration);

        // Getters
        const TaskID &get_id() const { return id_; }
        const Point &get_position() const { return position_; }
        const Point &get_head() const { return head_; }
        const Point &get_tail() const { return tail_; }
        double get_duration() const { return duration_; }
        bool is_completed() const { return completed_; }
        bool has_geometry() const { return has_geometry_; }
        const BoundingBox &get_bbox() const { return bbox_; }

        // Setters
        void set_completed(bool completed) { completed_ = completed; }
        void set_duration(double duration) { duration_ = duration; }

        /**
         * Get length of task (if geometric, distance from head to tail)
         */
        double get_length() const;

      private:
        TaskID id_;

        // Position data
        Point position_;    // Center or representative point
        Point head_;        // Start point (for geometric tasks)
        Point tail_;        // End point (for geometric tasks)
        bool has_geometry_; // True if head/tail are valid

        // Task properties
        double duration_; // Expected time to complete (seconds)
        bool completed_;

        // Spatial indexing
        BoundingBox bbox_;

        // Helper to compute bounding box
        void compute_bbox();
    };

} // namespace consens
