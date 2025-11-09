#include <consens/cbba/spatial_index.hpp>
#include <consens/task.hpp>

#include <iostream>
#include <spdlog/spdlog.h>

int main() {
    spdlog::set_level(spdlog::level::info);
    spdlog::info("=== Spatial Index Test ===\n");

    // Create spatial index
    consens::cbba::SpatialIndex spatial_index;
    spdlog::info("Created spatial index");

    // Create some tasks in a grid pattern
    spdlog::info("\n--- Adding Tasks ---");

    // Grid of 5x5 tasks
    int task_count = 0;
    for (int x = 0; x < 5; x++) {
        for (int y = 0; y < 5; y++) {
            std::string id = "task_" + std::to_string(task_count++);
            consens::Point pos(x * 10.0, y * 10.0);
            consens::Task task(id, pos, 5.0);
            spatial_index.insert(task);
        }
    }

    spdlog::info("Added {} tasks in 5x5 grid", spatial_index.size());

    // Add some geometric tasks (rows)
    spatial_index.insert(consens::Task("row_1", consens::Point(50.0, 10.0), consens::Point(50.0, 30.0), 10.0));
    spatial_index.insert(consens::Task("row_2", consens::Point(60.0, 10.0), consens::Point(60.0, 30.0), 10.0));
    spatial_index.insert(consens::Task("row_3", consens::Point(70.0, 10.0), consens::Point(70.0, 30.0), 10.0));

    spdlog::info("Added 3 geometric tasks (rows)");
    spdlog::info("Total tasks in index: {}", spatial_index.size());

    // Test get task
    spdlog::info("\n--- Testing Get Task ---");
    auto task_0 = spatial_index.get_task("task_0");
    if (task_0) {
        spdlog::info("task_0: pos=({:.1f}, {:.1f}), duration={:.1f}s", task_0->get_position().x,
                     task_0->get_position().y, task_0->get_duration());
    }

    auto row_1 = spatial_index.get_task("row_1");
    if (row_1) {
        spdlog::info("row_1: head=({:.1f}, {:.1f}), tail=({:.1f}, {:.1f}), length={:.2f}m", row_1->get_head().x,
                     row_1->get_head().y, row_1->get_tail().x, row_1->get_tail().y, row_1->get_length());
    }

    // Test nearest neighbor query
    spdlog::info("\n--- Testing Nearest Neighbor Query ---");
    consens::Point query_point(15.0, 15.0);
    spdlog::info("Query point: ({:.1f}, {:.1f})", query_point.x, query_point.y);

    auto nearest_5 = spatial_index.query_nearest(query_point, 5);
    spdlog::info("Found {} nearest tasks:", nearest_5.size());
    for (const auto &task_id : nearest_5) {
        auto task = spatial_index.get_task(task_id);
        if (task) {
            double dist = query_point.distance_to(task->get_position());
            spdlog::info("  - {}: pos=({:.1f}, {:.1f}), distance={:.2f}m", task_id, task->get_position().x,
                         task->get_position().y, dist);
        }
    }

    // Test radius query
    spdlog::info("\n--- Testing Radius Query ---");
    consens::Point query_center(20.0, 20.0);
    double radius = 15.0;
    spdlog::info("Query center: ({:.1f}, {:.1f}), radius: {:.1f}m", query_center.x, query_center.y, radius);

    auto in_radius = spatial_index.query_radius(query_center, radius);
    spdlog::info("Found {} tasks within radius:", in_radius.size());
    for (const auto &task_id : in_radius) {
        auto task = spatial_index.get_task(task_id);
        if (task) {
            double dist = query_center.distance_to(task->get_position());
            spdlog::info("  - {}: pos=({:.1f}, {:.1f}), distance={:.2f}m", task_id, task->get_position().x,
                         task->get_position().y, dist);
        }
    }

    // Test bounding box query
    spdlog::info("\n--- Testing Bounding Box Query ---");
    consens::BoundingBox bbox(5.0, 5.0, 35.0, 25.0);
    spdlog::info("Query box: ({:.1f}, {:.1f}) to ({:.1f}, {:.1f})", bbox.min_x, bbox.min_y, bbox.max_x, bbox.max_y);

    auto in_box = spatial_index.query_box(bbox);
    spdlog::info("Found {} tasks in box:", in_box.size());
    for (const auto &task_id : in_box) {
        auto task = spatial_index.get_task(task_id);
        if (task) {
            spdlog::info("  - {}: pos=({:.1f}, {:.1f})", task_id, task->get_position().x, task->get_position().y);
        }
    }

    // Test removal
    spdlog::info("\n--- Testing Task Removal ---");
    spatial_index.remove("task_12"); // Middle of grid
    spdlog::info("Removed task_12");

    // Query again in same area
    auto after_removal = spatial_index.query_radius(consens::Point(20.0, 20.0), 15.0);
    spdlog::info("After removal, {} tasks in radius", after_removal.size());

    spdlog::info("Has task_12: {}", spatial_index.has_task("task_12"));
    spdlog::info("Has task_0: {}", spatial_index.has_task("task_0"));

    // Test clear
    spdlog::info("\n--- Testing Clear ---");
    spdlog::info("Before clear: {} tasks", spatial_index.size());
    spatial_index.clear();
    spdlog::info("After clear: {} tasks", spatial_index.size());
    spdlog::info("Is empty: {}", spatial_index.empty());

    spdlog::info("\n=== Test Complete ===");
    return 0;
}
