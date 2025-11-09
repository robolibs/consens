#include <consens/consens.hpp>
#include <iostream>
#include <spdlog/spdlog.h>

int main() {
    spdlog::set_level(spdlog::level::info);
    spdlog::info("=== Consens Simple Test ===\n");

    // Create configuration for agent
    consens::Consens::Config config;
    config.agent_id = "robot_1";
    config.max_bundle_size = 5;
    config.spatial_query_radius = 50.0f;

    // Simple callbacks (no actual network for this test)
    config.send_message = [](const std::vector<uint8_t> &data) {
        spdlog::debug("Sending message of size: {}", data.size());
    };

    config.receive_messages = []() -> std::vector<std::vector<uint8_t>> {
        // No messages received in this simple test
        return {};
    };

    // Create consens instance
    spdlog::info("Creating consens instance for agent: {}", config.agent_id);
    consens::Consens consens_instance(config);

    // Update agent state
    spdlog::info("\n--- Updating Agent State ---");
    consens_instance.update_pose(10.0, 20.0, 0.5);
    consens_instance.update_velocity(2.5);

    auto pose = consens_instance.get_pose();
    spdlog::info("Agent pose: ({:.1f}, {:.1f}), heading: {:.2f}", pose.position.x, pose.position.y, pose.heading);
    spdlog::info("Agent velocity: {:.1f} m/s", consens_instance.get_velocity());

    // Add some tasks
    spdlog::info("\n--- Adding Tasks ---");

    consens_instance.add_task("task_1", consens::Point(15.0, 25.0), 10.0);
    spdlog::info("Added point task: task_1");

    consens_instance.add_task("task_2", consens::Point(20.0, 20.0), consens::Point(20.0, 40.0), 15.0);
    spdlog::info("Added geometric task: task_2");

    consens_instance.add_task("task_3", consens::Point(5.0, 30.0), 8.0);
    spdlog::info("Added point task: task_3");

    // Query tasks
    spdlog::info("\n--- Querying Tasks ---");
    auto all_tasks = consens_instance.get_all_tasks();
    spdlog::info("Total tasks: {}", all_tasks.size());

    for (const auto &task : all_tasks) {
        spdlog::info("  Task {}: pos=({:.1f}, {:.1f}), duration={:.1f}s, geometry={}", task.get_id(),
                     task.get_position().x, task.get_position().y, task.get_duration(),
                     task.has_geometry() ? "yes" : "no");
    }

    // Get specific task
    auto task2 = consens_instance.get_task("task_2");
    if (task2) {
        spdlog::info("\nTask 2 details:");
        spdlog::info("  Head: ({:.1f}, {:.1f})", task2->get_head().x, task2->get_head().y);
        spdlog::info("  Tail: ({:.1f}, {:.1f})", task2->get_tail().x, task2->get_tail().y);
        spdlog::info("  Length: {:.2f} m", task2->get_length());
    }

    // Run some ticks
    spdlog::info("\n--- Running Algorithm ---");
    for (int i = 0; i < 5; i++) {
        consens_instance.tick(0.1f);
    }

    auto stats = consens_instance.get_statistics();
    spdlog::info("After {} iterations:", stats.iteration_count);
    spdlog::info("  Bundle size: {}", stats.bundle_size);
    spdlog::info("  Total tasks: {}", stats.total_tasks);
    spdlog::info("  Converged: {}", stats.converged ? "yes" : "no");

    // Test bundle/path queries
    auto bundle = consens_instance.get_bundle();
    auto path = consens_instance.get_path();
    spdlog::info("\n  Current bundle: {} tasks", bundle.size());
    spdlog::info("  Current path: {} tasks", path.size());

    auto next_task = consens_instance.get_next_task();
    if (next_task) {
        spdlog::info("  Next task: {}", *next_task);
    } else {
        spdlog::info("  Next task: none");
    }

    // Test task completion
    spdlog::info("\n--- Testing Task Completion ---");
    consens_instance.mark_task_completed("task_1");
    spdlog::info("Marked task_1 as completed");

    auto task1 = consens_instance.get_task("task_1");
    if (task1) {
        spdlog::info("  Task 1 completed status: {}", task1->is_completed() ? "yes" : "no");
    }

    // Test task removal
    spdlog::info("\n--- Testing Task Removal ---");
    consens_instance.remove_task("task_3");
    spdlog::info("Removed task_3");

    all_tasks = consens_instance.get_all_tasks();
    spdlog::info("Remaining tasks: {}", all_tasks.size());

    // Test reset
    spdlog::info("\n--- Testing Reset ---");
    consens_instance.reset();
    stats = consens_instance.get_statistics();
    spdlog::info("After reset:");
    spdlog::info("  Iteration count: {}", stats.iteration_count);
    spdlog::info("  Bundle size: {}", stats.bundle_size);

    spdlog::info("\n=== Test Complete ===");
    return 0;
}
