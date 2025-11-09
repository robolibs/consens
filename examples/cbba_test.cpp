#include <consens/cbba/bid.hpp>
#include <consens/cbba/bundle.hpp>
#include <consens/cbba/cbba_agent.hpp>
#include <consens/cbba/types.hpp>

#include <iostream>
#include <spdlog/spdlog.h>

int main() {
    spdlog::set_level(spdlog::level::info);
    spdlog::info("=== CBBA Data Structures Test ===\n");

    // ========== Test Bids ==========
    spdlog::info("--- Testing Bids ---");

    consens::cbba::Bid bid1("agent_1", 10.5, 1.0);
    consens::cbba::Bid bid2("agent_2", 15.2, 1.0);
    consens::cbba::Bid bid3("agent_1", 10.5, 1.0);

    spdlog::info("bid1: agent={}, score={}, ts={}", bid1.agent_id, bid1.score, bid1.timestamp);
    spdlog::info("bid2: agent={}, score={}, ts={}", bid2.agent_id, bid2.score, bid2.timestamp);
    spdlog::info("bid1 > bid2: {}", bid1 > bid2);
    spdlog::info("bid2 > bid1: {}", bid2 > bid1);
    spdlog::info("bid1 == bid3: {}", bid1 == bid3);

    auto invalid_bid = consens::cbba::Bid::invalid();
    spdlog::info("Invalid bid valid: {}", invalid_bid.is_valid());
    spdlog::info("bid1 valid: {}", bid1.is_valid());

    // ========== Test Bundle ==========
    spdlog::info("\n--- Testing Bundle ---");

    consens::cbba::Bundle bundle(5);
    spdlog::info("Bundle capacity: {}", bundle.capacity());

    bundle.add("task_1");
    bundle.add("task_2");
    bundle.add("task_3");
    spdlog::info("Added 3 tasks, size: {}", bundle.size());

    spdlog::info("Contains task_1: {}", bundle.contains("task_1"));
    spdlog::info("Contains task_4: {}", bundle.contains("task_4"));

    bundle.remove("task_2");
    spdlog::info("After removing task_2, size: {}", bundle.size());

    spdlog::info("Bundle tasks:");
    for (const auto &task_id : bundle.get_tasks()) {
        spdlog::info("  - {}", task_id);
    }

    bundle.add("task_4");
    bundle.add("task_5");
    bundle.add("task_6");
    spdlog::info("After adding more, size: {} (is_full: {})", bundle.size(), bundle.is_full());

    bundle.add("task_7"); // Should not add, bundle is full
    spdlog::info("Tried to add when full, size: {}", bundle.size());

    // ========== Test Path ==========
    spdlog::info("\n--- Testing Path ---");

    consens::cbba::Path path;
    path.insert("task_A", 0);
    path.insert("task_B", 1);
    path.insert("task_C", 2);
    spdlog::info("Path size: {}", path.size());

    spdlog::info("Path tasks (execution order):");
    for (size_t i = 0; i < path.size(); i++) {
        spdlog::info("  {}. {}", i, path[i]);
    }

    // Insert in middle
    path.insert("task_X", 1);
    spdlog::info("\nAfter inserting task_X at position 1:");
    for (size_t i = 0; i < path.size(); i++) {
        spdlog::info("  {}. {}", i, path[i]);
    }

    spdlog::info("Position of task_B: {}", path.find_position("task_B"));
    spdlog::info("Position of task_Z (not in path): {}", path.find_position("task_Z"));

    path.remove("task_X");
    spdlog::info("After removing task_X, size: {}", path.size());

    spdlog::info("Front task: {}", path.front());

    // Test remove_from
    path.remove_from(2);
    spdlog::info("After remove_from(2), size: {}", path.size());

    // ========== Test CBBA Agent ==========
    spdlog::info("\n--- Testing CBBA Agent ---");

    consens::cbba::CBBAAgent agent("robot_1", 5);
    spdlog::info("Created agent: {}", agent.get_id());

    // Update state
    agent.update_pose(consens::Pose(10.0, 20.0, 0.5));
    agent.update_velocity(2.5);

    auto pose = agent.get_pose();
    spdlog::info("Agent pose: ({}, {}), heading: {}", pose.position.x, pose.position.y, pose.heading);
    spdlog::info("Agent velocity: {} m/s", agent.get_velocity());

    // Add tasks to bundle
    agent.add_to_bundle("row_1", -5.2, 0);
    agent.add_to_bundle("row_2", -8.1, 1);
    agent.add_to_bundle("row_3", -3.5, 2);

    spdlog::info("\nAgent bundle:");
    for (const auto &task_id : agent.get_bundle().get_tasks()) {
        spdlog::info("  - {}", task_id);
    }

    spdlog::info("\nAgent path:");
    for (size_t i = 0; i < agent.get_path().size(); i++) {
        spdlog::info("  {}. {}", i, agent.get_path()[i]);
    }

    // Check winning bids
    auto row1_bid = agent.get_winning_bid("row_1");
    spdlog::info("\nWinning bid for row_1: agent={}, score={}", row1_bid.agent_id, row1_bid.score);

    auto row1_winner = agent.get_winner("row_1");
    spdlog::info("Winner for row_1: {}", row1_winner);

    // Test timestamps
    agent.set_own_timestamp(5.0);
    agent.update_timestamp("robot_2", 4.5);
    spdlog::info("\nTimestamp for robot_1: {}", agent.get_timestamp("robot_1"));
    spdlog::info("Timestamp for robot_2: {}", agent.get_timestamp("robot_2"));

    // Test convergence
    agent.save_winners_for_convergence();
    agent.check_convergence();
    spdlog::info("\nAgent converged (first check): {}", agent.has_converged());

    // Change winners and check again
    agent.update_winning_bid("row_4", consens::cbba::Bid("robot_2", -2.0, 5.0));
    agent.check_convergence();
    spdlog::info("Agent converged (after change): {}", agent.has_converged());

    // Save again and check
    agent.save_winners_for_convergence();
    agent.check_convergence();
    spdlog::info("Agent converged (after save): {}", agent.has_converged());

    // Test reset task
    agent.reset_task("row_2");
    spdlog::info("\nAfter resetting row_2:");
    spdlog::info("  Bundle size: {}", agent.get_bundle().size());
    spdlog::info("  Path size: {}", agent.get_path().size());
    auto row2_winner = agent.get_winner("row_2");
    spdlog::info("  Winner for row_2: {}", row2_winner == consens::cbba::NO_AGENT ? "NO_AGENT" : row2_winner);

    // Test local bids
    agent.set_local_bid("row_5", -12.3);
    spdlog::info("\nLocal bid for row_5: {}", agent.get_local_bid("row_5"));
    spdlog::info("Local bid for row_6 (not set): {}", agent.get_local_bid("row_6"));

    spdlog::info("\n=== Test Complete ===");
    return 0;
}
