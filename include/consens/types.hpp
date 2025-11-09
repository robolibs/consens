#pragma once

#include <cmath>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace consens {

    // ============================================================================
    // Basic Types
    // ============================================================================

    /**
     * Unique identifier for an agent/robot
     */
    using AgentID = std::string;

    /**
     * Unique identifier for a task
     */
    using TaskID = std::string;

    /**
     * Score/utility value for tasks
     */
    using Score = double;

    /**
     * Timestamp for consensus protocol
     */
    using Timestamp = double;

    // ============================================================================
    // Geometry Types
    // ============================================================================

    /**
     * 2D Point
     */
    struct Point {
        double x;
        double y;

        Point() : x(0.0), y(0.0) {}
        Point(double x_, double y_) : x(x_), y(y_) {}

        /**
         * Calculate Euclidean distance to another point
         */
        double distance_to(const Point &other) const {
            double dx = x - other.x;
            double dy = y - other.y;
            return std::sqrt(dx * dx + dy * dy);
        }

        bool operator==(const Point &other) const { return x == other.x && y == other.y; }

        bool operator!=(const Point &other) const { return !(*this == other); }
    };

    /**
     * 2D Pose (position + heading)
     */
    struct Pose {
        Point position;
        double heading; // radians

        Pose() : position(), heading(0.0) {}
        Pose(const Point &pos, double h) : position(pos), heading(h) {}
        Pose(double x, double y, double h) : position(x, y), heading(h) {}
    };

    /**
     * Axis-aligned bounding box for spatial indexing
     */
    struct BoundingBox {
        double min_x;
        double min_y;
        double max_x;
        double max_y;

        BoundingBox() : min_x(0), min_y(0), max_x(0), max_y(0) {}

        BoundingBox(double min_x_, double min_y_, double max_x_, double max_y_)
            : min_x(min_x_), min_y(min_y_), max_x(max_x_), max_y(max_y_) {}

        /**
         * Create bounding box from a single point
         */
        static BoundingBox from_point(const Point &p, double padding = 0.0) {
            return BoundingBox(p.x - padding, p.y - padding, p.x + padding, p.y + padding);
        }

        /**
         * Create bounding box from two points (e.g., line segment)
         */
        static BoundingBox from_points(const Point &p1, const Point &p2, double padding = 0.0) {
            double min_x = std::min(p1.x, p2.x) - padding;
            double min_y = std::min(p1.y, p2.y) - padding;
            double max_x = std::max(p1.x, p2.x) + padding;
            double max_y = std::max(p1.y, p2.y) + padding;
            return BoundingBox(min_x, min_y, max_x, max_y);
        }

        /**
         * Check if point is inside bounding box
         */
        bool contains(const Point &p) const { return p.x >= min_x && p.x <= max_x && p.y >= min_y && p.y <= max_y; }

        /**
         * Check if bounding boxes intersect
         */
        bool intersects(const BoundingBox &other) const {
            return !(other.min_x > max_x || other.max_x < min_x || other.min_y > max_y || other.max_y < min_y);
        }

        /**
         * Get center point of bounding box
         */
        Point center() const { return Point((min_x + max_x) / 2.0, (min_y + max_y) / 2.0); }

        /**
         * Get width and height
         */
        double width() const { return max_x - min_x; }
        double height() const { return max_y - min_y; }
    };

    // ============================================================================
    // Communication Callbacks
    // ============================================================================

    /**
     * Callback for sending a message (broadcast to all neighbors)
     * User implements this to send via their communication system
     */
    using SendCallback = std::function<void(const std::vector<uint8_t> &)>;

    /**
     * Callback for receiving messages
     * User implements this to receive from their communication system
     * Returns list of received messages as byte arrays
     */
    using ReceiveCallback = std::function<std::vector<std::vector<uint8_t>>()>;

} // namespace consens
