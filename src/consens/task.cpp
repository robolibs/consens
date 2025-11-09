#include "consens/task.hpp"

namespace consens {

    Task::Task(const TaskID &id, const Point &position, double duration)
        : id_(id), position_(position), head_(position), tail_(position), has_geometry_(false), duration_(duration),
          completed_(false) {
        compute_bbox();
    }

    Task::Task(const TaskID &id, const Point &head, const Point &tail, double duration)
        : id_(id), position_((head.x + tail.x) / 2.0, (head.y + tail.y) / 2.0), head_(head), tail_(tail),
          has_geometry_(true), duration_(duration), completed_(false) {
        compute_bbox();
    }

    double Task::get_length() const {
        if (has_geometry_) {
            return head_.distance_to(tail_);
        }
        return 0.0;
    }

    void Task::compute_bbox() {
        if (has_geometry_) {
            // Bounding box from head to tail with small padding
            bbox_ = BoundingBox::from_points(head_, tail_, 1.0);
        } else {
            // Bounding box around single point with padding
            bbox_ = BoundingBox::from_point(position_, 5.0);
        }
    }

} // namespace consens
