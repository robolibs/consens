#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <consens/cbba/bundle_builder.hpp>
#include <consens/cbba/cbba_agent.hpp>
#include <consens/cbba/spatial_index.hpp>
#include <consens/task.hpp>

TEST_CASE("BundleBuilder - Basic Setup") {
    consens::cbba::SpatialIndex spatial_index;
    consens::cbba::BundleBuilder builder(&spatial_index);

    CHECK(builder.get_query_radius() == doctest::Approx(100.0));
    CHECK(builder.get_mode() == consens::cbba::BundleMode::ADD);
    CHECK(builder.get_metric() == consens::cbba::Metric::RPT);

    builder.set_query_radius(50.0);
    CHECK(builder.get_query_radius() == doctest::Approx(50.0));

    builder.set_mode(consens::cbba::BundleMode::FULLBUNDLE);
    CHECK(builder.get_mode() == consens::cbba::BundleMode::FULLBUNDLE);
}

TEST_CASE("BundleBuilder - ADD Mode Single Task") {
    consens::cbba::SpatialIndex spatial_index;
    consens::cbba::BundleBuilder builder(&spatial_index, consens::cbba::Metric::RPT, 100.0f,
                                         consens::cbba::BundleMode::ADD);

    // Setup agent at origin
    consens::cbba::CBBAAgent agent("robot_1", 5);
    agent.update_pose(consens::Pose(0.0, 0.0, 0.0));
    agent.update_velocity(2.0);

    // Add tasks nearby
    spatial_index.insert(consens::Task("task_1", consens::Point(10.0, 0.0), 5.0));
    spatial_index.insert(consens::Task("task_2", consens::Point(20.0, 0.0), 5.0));
    spatial_index.insert(consens::Task("task_3", consens::Point(30.0, 0.0), 5.0));

    std::vector<std::string> available_tasks = {"task_1", "task_2", "task_3"};

    SUBCASE("Add one task to empty bundle") {
        builder.build_bundle(agent, available_tasks);

        CHECK(agent.get_bundle().size() == 1);
        CHECK(agent.get_path().size() == 1);

        // Should pick task_1 (closest)
        CHECK(agent.get_bundle().contains("task_1"));
    }

    SUBCASE("Add multiple tasks one at a time") {
        // First task
        builder.build_bundle(agent, available_tasks);
        CHECK(agent.get_bundle().size() == 1);

        // Second task
        builder.build_bundle(agent, available_tasks);
        CHECK(agent.get_bundle().size() == 2);

        // Third task
        builder.build_bundle(agent, available_tasks);
        CHECK(agent.get_bundle().size() == 3);
    }
}

TEST_CASE("BundleBuilder - FULLBUNDLE Mode") {
    consens::cbba::SpatialIndex spatial_index;
    consens::cbba::BundleBuilder builder(&spatial_index, consens::cbba::Metric::RPT, 100.0f,
                                         consens::cbba::BundleMode::FULLBUNDLE);

    // Setup agent
    consens::cbba::CBBAAgent agent("robot_1", 5);
    agent.update_pose(consens::Pose(0.0, 0.0, 0.0));
    agent.update_velocity(2.0);

    // Add tasks
    for (int i = 0; i < 8; i++) {
        std::string id = "task_" + std::to_string(i);
        spatial_index.insert(consens::Task(id, consens::Point(i * 10.0, 0.0), 5.0));
    }

    std::vector<std::string> available_tasks;
    for (int i = 0; i < 8; i++) {
        available_tasks.push_back("task_" + std::to_string(i));
    }

    SUBCASE("Fill bundle to capacity") {
        builder.build_bundle(agent, available_tasks);

        // Should fill to capacity (5)
        CHECK(agent.get_bundle().size() == 5);
        CHECK(agent.get_bundle().is_full());
    }
}

TEST_CASE("BundleBuilder - Spatial Filtering") {
    consens::cbba::SpatialIndex spatial_index;
    consens::cbba::BundleBuilder builder(&spatial_index, consens::cbba::Metric::RPT, 30.0f,
                                         consens::cbba::BundleMode::ADD);

    // Setup agent at origin
    consens::cbba::CBBAAgent agent("robot_1", 10);
    agent.update_pose(consens::Pose(0.0, 0.0, 0.0));
    agent.update_velocity(2.0);

    // Add tasks: some nearby, some far
    spatial_index.insert(consens::Task("near_1", consens::Point(10.0, 0.0), 5.0));
    spatial_index.insert(consens::Task("near_2", consens::Point(20.0, 0.0), 5.0));
    spatial_index.insert(consens::Task("far_1", consens::Point(100.0, 0.0), 5.0));
    spatial_index.insert(consens::Task("far_2", consens::Point(200.0, 0.0), 5.0));

    std::vector<std::string> available_tasks = {"near_1", "near_2", "far_1", "far_2"};

    SUBCASE("Only considers tasks within radius") {
        builder.build_bundle(agent, available_tasks);

        CHECK(agent.get_bundle().size() == 1);

        // Should only pick from near tasks (within 30m)
        bool picked_near = agent.get_bundle().contains("near_1") || agent.get_bundle().contains("near_2");
        CHECK(picked_near);

        // Should not pick far tasks
        CHECK_FALSE(agent.get_bundle().contains("far_1"));
        CHECK_FALSE(agent.get_bundle().contains("far_2"));
    }

    SUBCASE("Can increase radius to include more tasks") {
        builder.set_query_radius(150.0f);
        builder.build_bundle(agent, available_tasks);

        CHECK(agent.get_bundle().size() == 1);

        // Could be near or far_1
        bool valid_pick = agent.get_bundle().contains("near_1") || agent.get_bundle().contains("near_2") ||
                          agent.get_bundle().contains("far_1");
        CHECK(valid_pick);
    }
}

TEST_CASE("BundleBuilder - Bidding Logic") {
    consens::cbba::SpatialIndex spatial_index;
    consens::cbba::BundleBuilder builder(&spatial_index);

    // Setup agent
    consens::cbba::CBBAAgent agent("robot_1", 10);
    agent.update_pose(consens::Pose(0.0, 0.0, 0.0));
    agent.update_velocity(2.0);

    // Add task
    spatial_index.insert(consens::Task("task_1", consens::Point(10.0, 0.0), 5.0));

    std::vector<std::string> available_tasks = {"task_1"};

    SUBCASE("Bids on unassigned task") {
        builder.build_bundle(agent, available_tasks);

        CHECK(agent.get_bundle().contains("task_1"));

        // Check that winning bid is set
        auto bid = agent.get_winning_bid("task_1");
        CHECK(bid.is_valid());
        CHECK(bid.agent_id == "robot_1");
    }

    SUBCASE("Does not bid if another agent has better bid") {
        // Simulate another agent with better bid
        consens::cbba::Bid better_bid("robot_2", 100.0, 1.0); // Much better score
        agent.update_winning_bid("task_1", better_bid);

        builder.build_bundle(agent, available_tasks);

        // Should not add to bundle because other agent has better bid
        CHECK_FALSE(agent.get_bundle().contains("task_1"));
    }
}

TEST_CASE("BundleBuilder - Greedy Selection") {
    consens::cbba::SpatialIndex spatial_index;
    consens::cbba::BundleBuilder builder(&spatial_index, consens::cbba::Metric::RPT, 100.0f,
                                         consens::cbba::BundleMode::FULLBUNDLE);

    // Setup agent at origin
    consens::cbba::CBBAAgent agent("robot_1", 3);
    agent.update_pose(consens::Pose(0.0, 0.0, 0.0));
    agent.update_velocity(2.0);

    // Add tasks at different distances
    spatial_index.insert(consens::Task("close", consens::Point(5.0, 0.0), 5.0));
    spatial_index.insert(consens::Task("medium", consens::Point(15.0, 0.0), 5.0));
    spatial_index.insert(consens::Task("far", consens::Point(30.0, 0.0), 5.0));

    std::vector<std::string> available_tasks = {"close", "medium", "far"};

    SUBCASE("Greedily picks best tasks") {
        builder.build_bundle(agent, available_tasks);

        // Should fill bundle with 3 tasks
        CHECK(agent.get_bundle().size() == 3);

        // All tasks should be in bundle (ordered by efficiency)
        CHECK(agent.get_bundle().contains("close"));
        CHECK(agent.get_bundle().contains("medium"));
        CHECK(agent.get_bundle().contains("far"));

        // Path should be ordered (close -> medium -> far is optimal)
        const auto &path = agent.get_path();
        CHECK(path[0] == "close");
    }
}

TEST_CASE("BundleBuilder - Empty Available Tasks") {
    consens::cbba::SpatialIndex spatial_index;
    consens::cbba::BundleBuilder builder(&spatial_index);

    consens::cbba::CBBAAgent agent("robot_1", 5);
    agent.update_pose(consens::Pose(0.0, 0.0, 0.0));
    agent.update_velocity(2.0);

    // Add tasks to spatial index
    spatial_index.insert(consens::Task("task_1", consens::Point(10.0, 0.0), 5.0));

    // But empty available list
    std::vector<std::string> available_tasks;

    SUBCASE("Handles empty available tasks") {
        builder.build_bundle(agent, available_tasks);

        CHECK(agent.get_bundle().size() == 0);
        CHECK(agent.get_bundle().empty());
    }
}

TEST_CASE("BundleBuilder - Bundle Already Full") {
    consens::cbba::SpatialIndex spatial_index;
    consens::cbba::BundleBuilder builder(&spatial_index);

    consens::cbba::CBBAAgent agent("robot_1", 2); // Small capacity
    agent.update_pose(consens::Pose(0.0, 0.0, 0.0));
    agent.update_velocity(2.0);

    // Add tasks
    for (int i = 0; i < 5; i++) {
        std::string id = "task_" + std::to_string(i);
        spatial_index.insert(consens::Task(id, consens::Point(i * 10.0, 0.0), 5.0));
    }

    std::vector<std::string> available_tasks;
    for (int i = 0; i < 5; i++) {
        available_tasks.push_back("task_" + std::to_string(i));
    }

    SUBCASE("Stops when bundle is full") {
        // Fill bundle
        builder.build_bundle(agent, available_tasks);
        builder.build_bundle(agent, available_tasks);

        CHECK(agent.get_bundle().size() == 2);
        CHECK(agent.get_bundle().is_full());

        // Try to add another - should not add
        size_t size_before = agent.get_bundle().size();
        builder.build_bundle(agent, available_tasks);
        CHECK(agent.get_bundle().size() == size_before);
    }
}
