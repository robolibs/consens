#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <consens/cbba/cbba_agent.hpp>
#include <consens/cbba/scorer.hpp>
#include <consens/cbba/spatial_index.hpp>
#include <consens/task.hpp>

TEST_CASE("TaskScorer - Basic Setup") {
    consens::cbba::TaskScorer scorer(consens::cbba::Metric::RPT);

    CHECK(scorer.get_metric() == consens::cbba::Metric::RPT);

    scorer.set_metric(consens::cbba::Metric::TDR);
    CHECK(scorer.get_metric() == consens::cbba::Metric::TDR);
}

TEST_CASE("TaskScorer - Empty Path") {
    consens::cbba::TaskScorer scorer;
    consens::cbba::CBBAAgent agent("robot_1", 5);
    consens::cbba::SpatialIndex spatial_index;
    consens::cbba::Path empty_path;

    agent.update_pose(consens::Pose(0.0, 0.0, 0.0));
    agent.update_velocity(2.0);

    SUBCASE("Evaluate empty path") {
        auto score = scorer.evaluate_path(agent, empty_path, spatial_index);
        CHECK(score == 0.0);
    }
}

TEST_CASE("TaskScorer - Single Task Path") {
    consens::cbba::TaskScorer scorer(consens::cbba::Metric::RPT);
    consens::cbba::CBBAAgent agent("robot_1", 5);
    consens::cbba::SpatialIndex spatial_index;

    // Setup agent
    agent.update_pose(consens::Pose(0.0, 0.0, 0.0));
    agent.update_velocity(2.0); // 2 m/s

    // Add task 10m away, taking 5s to complete
    consens::Task task("task_1", consens::Point(10.0, 0.0), 5.0);
    spatial_index.insert(task);

    consens::cbba::Path path;
    path.insert("task_1", 0);

    SUBCASE("Evaluate single task") {
        auto score = scorer.evaluate_path(agent, path, spatial_index);

        // Expected: travel_time = 10m / 2m/s = 5s, task_time = 5s, total = 10s
        // RPT score = -10s
        CHECK(score == doctest::Approx(-10.0));
    }
}

TEST_CASE("TaskScorer - Multiple Tasks Path") {
    consens::cbba::TaskScorer scorer(consens::cbba::Metric::RPT);
    consens::cbba::CBBAAgent agent("robot_1", 10);
    consens::cbba::SpatialIndex spatial_index;

    // Setup agent at origin
    agent.update_pose(consens::Pose(0.0, 0.0, 0.0));
    agent.update_velocity(2.0);

    // Add tasks in a line
    spatial_index.insert(consens::Task("task_1", consens::Point(10.0, 0.0), 5.0));
    spatial_index.insert(consens::Task("task_2", consens::Point(20.0, 0.0), 5.0));
    spatial_index.insert(consens::Task("task_3", consens::Point(30.0, 0.0), 5.0));

    consens::cbba::Path path;
    path.insert("task_1", 0);
    path.insert("task_2", 1);
    path.insert("task_3", 2);

    SUBCASE("Evaluate path in order") {
        auto score = scorer.evaluate_path(agent, path, spatial_index);

        // travel(0->10) = 5s, task1 = 5s
        // travel(10->20) = 5s, task2 = 5s
        // travel(20->30) = 5s, task3 = 5s
        // Total = 30s, RPT = -30s
        CHECK(score == doctest::Approx(-30.0));
    }

    SUBCASE("Path with different order") {
        consens::cbba::Path path2;
        path2.insert("task_3", 0);
        path2.insert("task_1", 1);
        path2.insert("task_2", 2);

        auto score = scorer.evaluate_path(agent, path2, spatial_index);

        // travel(0->30) = 15s, task3 = 5s
        // travel(30->10) = 10s, task1 = 5s
        // travel(10->20) = 5s, task2 = 5s
        // Total = 45s, RPT = -45s
        CHECK(score == doctest::Approx(-45.0));
    }
}

TEST_CASE("TaskScorer - Marginal Gain") {
    consens::cbba::TaskScorer scorer(consens::cbba::Metric::RPT);
    consens::cbba::CBBAAgent agent("robot_1", 10);
    consens::cbba::SpatialIndex spatial_index;

    agent.update_pose(consens::Pose(0.0, 0.0, 0.0));
    agent.update_velocity(2.0);

    // Setup tasks
    spatial_index.insert(consens::Task("task_1", consens::Point(10.0, 0.0), 5.0));
    spatial_index.insert(consens::Task("task_2", consens::Point(20.0, 0.0), 5.0));
    spatial_index.insert(consens::Task("task_new", consens::Point(15.0, 0.0), 5.0));

    // Existing path: task_1 -> task_2
    consens::cbba::Path path;
    path.insert("task_1", 0);
    path.insert("task_2", 1);

    SUBCASE("Insert at end") {
        auto task_opt = spatial_index.get_task("task_new");
        REQUIRE(task_opt.has_value());

        auto gain = scorer.compute_marginal_gain(agent, *task_opt, path, 2, spatial_index);

        // Original: 0->10 (5s) + task1 (5s) + 10->20 (5s) + task2 (5s) = 20s, score = -20s
        // With new: ... + 20->15 (2.5s) + task_new (5s) = 27.5s, score = -27.5s
        // Gain = -27.5 - (-20) = -7.5
        CHECK(gain == doctest::Approx(-7.5));
    }

    SUBCASE("Insert in middle") {
        auto task_opt = spatial_index.get_task("task_new");
        REQUIRE(task_opt.has_value());

        auto gain = scorer.compute_marginal_gain(agent, *task_opt, path, 1, spatial_index);

        // Original: 20s, score = -20s
        // With new in middle: 0->10 (5s) + task1 (5s) + 10->15 (2.5s) + task_new (5s) + 15->20 (2.5s) + task2 (5s) =
        // 25s Gain = -25 - (-20) = -5
        CHECK(gain == doctest::Approx(-5.0));
    }
}

TEST_CASE("TaskScorer - Find Optimal Insertion") {
    consens::cbba::TaskScorer scorer(consens::cbba::Metric::RPT);
    consens::cbba::CBBAAgent agent("robot_1", 10);
    consens::cbba::SpatialIndex spatial_index;

    agent.update_pose(consens::Pose(0.0, 0.0, 0.0));
    agent.update_velocity(2.0);

    // Setup tasks
    spatial_index.insert(consens::Task("task_1", consens::Point(10.0, 0.0), 5.0));
    spatial_index.insert(consens::Task("task_2", consens::Point(30.0, 0.0), 5.0));
    spatial_index.insert(consens::Task("task_mid", consens::Point(20.0, 0.0), 5.0));

    // Path: task_1 -> task_2
    consens::cbba::Path path;
    path.insert("task_1", 0);
    path.insert("task_2", 1);

    SUBCASE("Find best position for middle task") {
        auto task_opt = spatial_index.get_task("task_mid");
        REQUIRE(task_opt.has_value());

        auto [best_score, best_pos] = scorer.find_optimal_insertion(agent, *task_opt, path, spatial_index);

        // Best position should be in the middle (position 1)
        CHECK(best_pos == 1);
        CHECK(best_score > consens::cbba::MIN_SCORE);
    }
}

TEST_CASE("TaskScorer - Geometric Tasks") {
    consens::cbba::TaskScorer scorer(consens::cbba::Metric::RPT);
    consens::cbba::CBBAAgent agent("robot_1", 10);
    consens::cbba::SpatialIndex spatial_index;

    agent.update_pose(consens::Pose(0.0, 0.0, 0.0));
    agent.update_velocity(2.0);

    // Add geometric task (row)
    consens::Point head(10.0, 0.0);
    consens::Point tail(10.0, 20.0);
    consens::Task row("row_1", head, tail, 10.0);
    spatial_index.insert(row);

    consens::cbba::Path path;
    path.insert("row_1", 0);

    SUBCASE("Evaluate geometric task") {
        auto score = scorer.evaluate_path(agent, path, spatial_index);

        // Travel to center: sqrt(10^2 + 10^2) ≈ 14.14m, time ≈ 7.07s
        // Task time: 10s
        // Total: ~17.07s, RPT = ~-17.07s
        CHECK(score == doctest::Approx(-17.07).epsilon(0.01));
    }
}

TEST_CASE("TaskScorer - TDR Metric") {
    consens::cbba::TaskScorer scorer(consens::cbba::Metric::TDR, 0.9);
    consens::cbba::CBBAAgent agent("robot_1", 10);
    consens::cbba::SpatialIndex spatial_index;

    agent.update_pose(consens::Pose(0.0, 0.0, 0.0));
    agent.update_velocity(2.0);

    // Add tasks
    spatial_index.insert(consens::Task("task_1", consens::Point(10.0, 0.0), 5.0));
    spatial_index.insert(consens::Task("task_2", consens::Point(20.0, 0.0), 5.0));

    consens::cbba::Path path;
    path.insert("task_1", 0);
    path.insert("task_2", 1);

    SUBCASE("TDR gives positive reward") {
        auto score = scorer.evaluate_path(agent, path, spatial_index);

        // TDR should give positive values (discounted rewards)
        CHECK(score > 0.0);
        CHECK(score < 2.0); // Should be less than 2 (max would be 2 tasks without discount)
    }

    SUBCASE("Earlier tasks have higher weight") {
        // Path 1: task_1 then task_2
        auto score1 = scorer.evaluate_path(agent, path, spatial_index);

        // Path 2: task_2 then task_1
        consens::cbba::Path path2;
        path2.insert("task_2", 0);
        path2.insert("task_1", 1);
        auto score2 = scorer.evaluate_path(agent, path2, spatial_index);

        // Path 1 should be better (shorter total time)
        CHECK(score1 > score2);
    }
}
