#include "consens/cbba/spatial_index.hpp"

#include <algorithm>

namespace consens::cbba {

    SpatialIndex::SpatialIndex() : rtree_(std::make_unique<RTree>()) {}

    SpatialIndex::~SpatialIndex() = default;

    void SpatialIndex::insert(const Task &task) {
        // Store task in map
        tasks_[task.get_id()] = task;

        // Get pointer to the TaskID in the map (stable across inserts)
        const TaskID *id_ptr = &tasks_.find(task.get_id())->first;

        // Insert into R-tree
        BoostBox box = task_to_boost_box(task);
        rtree_->insert(std::make_pair(box, id_ptr));
    }

    void SpatialIndex::remove(const TaskID &task_id) {
        auto it = tasks_.find(task_id);
        if (it == tasks_.end()) {
            return;
        }

        // Remove from R-tree
        const TaskID *id_ptr = &it->first;
        BoostBox box = task_to_boost_box(it->second);

        // Remove all entries with this ID
        rtree_->remove(std::make_pair(box, id_ptr));

        // Remove from map
        tasks_.erase(it);
    }

    void SpatialIndex::clear() {
        rtree_->clear();
        tasks_.clear();
    }

    std::vector<TaskID> SpatialIndex::query_nearest(const Point &position, size_t k) const {
        std::vector<TaskID> result;
        BoostPoint query_point = to_boost_point(position);

        // Query k nearest neighbors
        std::vector<RTreeValue> nearest;
        rtree_->query(bgi::nearest(query_point, k), std::back_inserter(nearest));

        // Extract task IDs
        result.reserve(nearest.size());
        for (const auto &value : nearest) {
            result.push_back(*value.second);
        }

        return result;
    }

    std::vector<TaskID> SpatialIndex::query_radius(const Point &position, double radius) const {
        std::vector<TaskID> result;
        BoostPoint query_point = to_boost_point(position);

        // Create a bounding box for the radius query
        BoostBox query_box(BoostPoint(position.x - radius, position.y - radius),
                           BoostPoint(position.x + radius, position.y + radius));

        // Query all tasks intersecting the box
        std::vector<RTreeValue> candidates;
        rtree_->query(bgi::intersects(query_box), std::back_inserter(candidates));

        // Filter by actual distance
        result.reserve(candidates.size());
        for (const auto &value : candidates) {
            const Task &task = tasks_.at(*value.second);
            double dist = position.distance_to(task.get_position());
            if (dist <= radius) {
                result.push_back(*value.second);
            }
        }

        return result;
    }

    std::vector<TaskID> SpatialIndex::query_box(const BoundingBox &bbox) const {
        std::vector<TaskID> result;
        BoostBox query_box = to_boost_box(bbox);

        // Query all tasks intersecting the box
        std::vector<RTreeValue> found;
        rtree_->query(bgi::intersects(query_box), std::back_inserter(found));

        // Extract task IDs
        result.reserve(found.size());
        for (const auto &value : found) {
            result.push_back(*value.second);
        }

        return result;
    }

    std::optional<Task> SpatialIndex::get_task(const TaskID &id) const {
        auto it = tasks_.find(id);
        if (it != tasks_.end()) {
            return it->second;
        }
        return std::nullopt;
    }

    bool SpatialIndex::has_task(const TaskID &id) const { return tasks_.find(id) != tasks_.end(); }

    size_t SpatialIndex::size() const { return tasks_.size(); }

    bool SpatialIndex::empty() const { return tasks_.empty(); }

    std::vector<TaskID> SpatialIndex::get_all_task_ids() const {
        std::vector<TaskID> result;
        result.reserve(tasks_.size());
        for (const auto &[id, task] : tasks_) {
            result.push_back(id);
        }
        return result;
    }

    std::vector<Task> SpatialIndex::get_all_tasks() const {
        std::vector<Task> result;
        result.reserve(tasks_.size());
        for (const auto &[id, task] : tasks_) {
            result.push_back(task);
        }
        return result;
    }

    BoostPoint SpatialIndex::to_boost_point(const Point &p) { return BoostPoint(p.x, p.y); }

    BoostBox SpatialIndex::to_boost_box(const BoundingBox &bbox) {
        return BoostBox(BoostPoint(bbox.min_x, bbox.min_y), BoostPoint(bbox.max_x, bbox.max_y));
    }

    BoostBox SpatialIndex::task_to_boost_box(const Task &task) {
        const BoundingBox &bbox = task.get_bbox();
        return to_boost_box(bbox);
    }

} // namespace consens::cbba
