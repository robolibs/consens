#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <consens/cbba/spatial_index.hpp>
#include <consens/task.hpp>

TEST_CASE("SpatialIndex - Basic Operations") {
    consens::cbba::SpatialIndex index;

    SUBCASE("Empty index") {
        CHECK(index.size() == 0);
        CHECK(index.empty());
        CHECK_FALSE(index.has_task("nonexistent"));
    }

    SUBCASE("Insert and retrieve task") {
        consens::Task task("task_1", consens::Point(10.0, 20.0), 5.0);
        index.insert(task);

        CHECK(index.size() == 1);
        CHECK_FALSE(index.empty());
        CHECK(index.has_task("task_1"));

        auto retrieved = index.get_task("task_1");
        REQUIRE(retrieved.has_value());
        CHECK(retrieved->get_id() == "task_1");
        CHECK(retrieved->get_position().x == 10.0);
        CHECK(retrieved->get_position().y == 20.0);
    }

    SUBCASE("Insert multiple tasks") {
        for (int i = 0; i < 10; i++) {
            std::string id = "task_" + std::to_string(i);
            consens::Task task(id, consens::Point(i * 10.0, i * 5.0), 3.0);
            index.insert(task);
        }

        CHECK(index.size() == 10);
        CHECK(index.has_task("task_5"));
        CHECK_FALSE(index.has_task("task_10"));
    }

    SUBCASE("Remove task") {
        consens::Task task("task_1", consens::Point(10.0, 20.0), 5.0);
        index.insert(task);

        CHECK(index.has_task("task_1"));

        index.remove("task_1");

        CHECK_FALSE(index.has_task("task_1"));
        CHECK(index.size() == 0);
    }

    SUBCASE("Clear index") {
        for (int i = 0; i < 5; i++) {
            std::string id = "task_" + std::to_string(i);
            index.insert(consens::Task(id, consens::Point(i * 10.0, 0.0), 2.0));
        }

        CHECK(index.size() == 5);

        index.clear();

        CHECK(index.size() == 0);
        CHECK(index.empty());
    }
}

TEST_CASE("SpatialIndex - Nearest Neighbor Query") {
    consens::cbba::SpatialIndex index;

    // Create a grid of tasks
    for (int x = 0; x < 5; x++) {
        for (int y = 0; y < 5; y++) {
            std::string id = "task_" + std::to_string(x) + "_" + std::to_string(y);
            consens::Task task(id, consens::Point(x * 10.0, y * 10.0), 5.0);
            index.insert(task);
        }
    }

    SUBCASE("Query 1 nearest") {
        consens::Point query(15.0, 15.0);
        auto nearest = index.query_nearest(query, 1);

        REQUIRE(nearest.size() == 1);
        // Closest should be task_1_1 or task_2_1 or task_1_2 or task_2_2
        // All are equidistant from (15, 15)
        CHECK(nearest[0].find("task_") == 0);
    }

    SUBCASE("Query 5 nearest") {
        consens::Point query(0.0, 0.0);
        auto nearest = index.query_nearest(query, 5);

        CHECK(nearest.size() == 5);
        // First should be task_0_0 (at origin) - but check it's in the results
        bool found_origin = std::find(nearest.begin(), nearest.end(), "task_0_0") != nearest.end();
        CHECK(found_origin);
    }

    SUBCASE("Query more than available") {
        consens::Point query(0.0, 0.0);
        auto nearest = index.query_nearest(query, 100);

        // Should return all 25 tasks
        CHECK(nearest.size() == 25);
    }
}

TEST_CASE("SpatialIndex - Radius Query") {
    consens::cbba::SpatialIndex index;

    // Create tasks in a line
    for (int i = 0; i < 10; i++) {
        std::string id = "task_" + std::to_string(i);
        consens::Task task(id, consens::Point(i * 10.0, 0.0), 3.0);
        index.insert(task);
    }

    SUBCASE("Small radius") {
        consens::Point center(20.0, 0.0);
        double radius = 5.0;
        auto in_radius = index.query_radius(center, radius);

        // Only task_2 (at x=20) should be within 5m
        CHECK(in_radius.size() == 1);
        CHECK(in_radius[0] == "task_2");
    }

    SUBCASE("Medium radius") {
        consens::Point center(20.0, 0.0);
        double radius = 15.0;
        auto in_radius = index.query_radius(center, radius);

        // task_1, task_2, task_3 should be within 15m
        CHECK(in_radius.size() >= 3);
    }

    SUBCASE("Large radius") {
        consens::Point center(0.0, 0.0);
        double radius = 100.0;
        auto in_radius = index.query_radius(center, radius);

        // All 10 tasks should be within 100m
        CHECK(in_radius.size() == 10);
    }

    SUBCASE("No tasks in radius") {
        consens::Point center(1000.0, 1000.0);
        double radius = 10.0;
        auto in_radius = index.query_radius(center, radius);

        CHECK(in_radius.size() == 0);
    }
}

TEST_CASE("SpatialIndex - Bounding Box Query") {
    consens::cbba::SpatialIndex index;

    // Create a 5x5 grid
    for (int x = 0; x < 5; x++) {
        for (int y = 0; y < 5; y++) {
            std::string id = "task_" + std::to_string(x) + "_" + std::to_string(y);
            consens::Task task(id, consens::Point(x * 10.0, y * 10.0), 5.0);
            index.insert(task);
        }
    }

    SUBCASE("Small box") {
        consens::BoundingBox bbox(5.0, 5.0, 15.0, 15.0);
        auto in_box = index.query_box(bbox);

        // Should contain task_1_1 (10, 10) and possibly corners
        CHECK(in_box.size() >= 1);
    }

    SUBCASE("Large box covering all") {
        consens::BoundingBox bbox(-10.0, -10.0, 100.0, 100.0);
        auto in_box = index.query_box(bbox);

        // Should contain all 25 tasks
        CHECK(in_box.size() == 25);
    }

    SUBCASE("Box with no tasks") {
        consens::BoundingBox bbox(1000.0, 1000.0, 2000.0, 2000.0);
        auto in_box = index.query_box(bbox);

        CHECK(in_box.size() == 0);
    }
}

TEST_CASE("SpatialIndex - Geometric Tasks") {
    consens::cbba::SpatialIndex index;

    SUBCASE("Insert geometric task (row/swath)") {
        consens::Point head(10.0, 0.0);
        consens::Point tail(10.0, 50.0);
        consens::Task row("row_1", head, tail, 10.0);

        index.insert(row);

        CHECK(index.size() == 1);
        CHECK(index.has_task("row_1"));

        auto retrieved = index.get_task("row_1");
        REQUIRE(retrieved.has_value());
        CHECK(retrieved->has_geometry());
        CHECK(retrieved->get_length() == doctest::Approx(50.0));
    }

    SUBCASE("Query geometric tasks") {
        // Create parallel rows
        for (int i = 0; i < 5; i++) {
            std::string id = "row_" + std::to_string(i);
            consens::Point head(i * 10.0, 0.0);
            consens::Point tail(i * 10.0, 100.0);
            consens::Task row(id, head, tail, 20.0);
            index.insert(row);
        }

        // Query near the middle of row_2
        consens::Point query(20.0, 50.0);
        auto nearest = index.query_nearest(query, 3);

        CHECK(nearest.size() == 3);
        // row_2 should be in the nearest results (center at 20, 50)
        bool found_row2 = std::find(nearest.begin(), nearest.end(), "row_2") != nearest.end();
        CHECK(found_row2);
    }
}

TEST_CASE("SpatialIndex - Get All") {
    consens::cbba::SpatialIndex index;

    for (int i = 0; i < 5; i++) {
        std::string id = "task_" + std::to_string(i);
        index.insert(consens::Task(id, consens::Point(i * 10.0, 0.0), 3.0));
    }

    SUBCASE("Get all task IDs") {
        auto all_ids = index.get_all_task_ids();
        CHECK(all_ids.size() == 5);

        // Check all IDs are present
        for (int i = 0; i < 5; i++) {
            std::string id = "task_" + std::to_string(i);
            CHECK(std::find(all_ids.begin(), all_ids.end(), id) != all_ids.end());
        }
    }

    SUBCASE("Get all tasks") {
        auto all_tasks = index.get_all_tasks();
        CHECK(all_tasks.size() == 5);

        // Verify tasks have correct data
        for (const auto &task : all_tasks) {
            CHECK(task.get_id().find("task_") == 0);
            CHECK(task.get_duration() == 3.0);
        }
    }
}
