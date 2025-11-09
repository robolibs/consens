#pragma once

#include "../task.hpp"
#include "types.hpp"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <map>
#include <memory>
#include <vector>

namespace consens::cbba {

    // Boost geometry types for R-tree
    namespace bg = boost::geometry;
    namespace bgi = boost::geometry::index;

    using BoostPoint = bg::model::point<double, 2, bg::cs::cartesian>;
    using BoostBox = bg::model::box<BoostPoint>;

    /**
     * Value stored in R-tree: (BoundingBox, pointer to TaskID string)
     * We use a pointer to avoid copies
     */
    using RTreeValue = std::pair<BoostBox, const TaskID *>;

    /**
     * R-tree type using quadratic split algorithm
     */
    using RTree = bgi::rtree<RTreeValue, bgi::quadratic<16>>;

    /**
     * Spatial index for efficient task queries using R-tree
     * Wraps boost::geometry R-tree for spatial operations
     */
    class SpatialIndex {
      private:
        std::unique_ptr<RTree> rtree_;
        std::map<TaskID, Task> tasks_;

      public:
        SpatialIndex();
        ~SpatialIndex();

        /**
         * Insert a task into the spatial index
         */
        void insert(const Task &task);

        /**
         * Remove a task from the spatial index
         */
        void remove(const TaskID &task_id);

        /**
         * Clear all tasks
         */
        void clear();

        /**
         * Query K nearest tasks to a point
         * @param position Query point
         * @param k Number of nearest tasks to return
         * @return Vector of task IDs, sorted by distance (closest first)
         */
        std::vector<TaskID> query_nearest(const Point &position, size_t k) const;

        /**
         * Query tasks within a radius
         * @param position Center point
         * @param radius Search radius (meters)
         * @return Vector of task IDs within radius
         */
        std::vector<TaskID> query_radius(const Point &position, double radius) const;

        /**
         * Query tasks within a bounding box
         * @param bbox Bounding box to query
         * @return Vector of task IDs within bbox
         */
        std::vector<TaskID> query_box(const BoundingBox &bbox) const;

        /**
         * Get task by ID
         * @return Task if found, std::nullopt otherwise
         */
        std::optional<Task> get_task(const TaskID &id) const;

        /**
         * Check if task exists
         */
        bool has_task(const TaskID &id) const;

        /**
         * Get number of tasks in index
         */
        size_t size() const;

        /**
         * Check if index is empty
         */
        bool empty() const;

        /**
         * Get all task IDs
         */
        std::vector<TaskID> get_all_task_ids() const;

        /**
         * Get all tasks
         */
        std::vector<Task> get_all_tasks() const;

      private:
        /**
         * Convert consens Point to boost Point
         */
        static BoostPoint to_boost_point(const Point &p);

        /**
         * Convert consens BoundingBox to boost Box
         */
        static BoostBox to_boost_box(const BoundingBox &bbox);

        /**
         * Create boost Box from task
         */
        static BoostBox task_to_boost_box(const Task &task);
    };

} // namespace consens::cbba
