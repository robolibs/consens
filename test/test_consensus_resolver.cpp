#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <consens/cbba/cbba_agent.hpp>
#include <consens/cbba/consensus_resolver.hpp>
#include <consens/cbba/messages.hpp>

using namespace consens::cbba;

TEST_CASE("ConsensusResolver - Basic Setup") {
    ConsensusResolver resolver;
    CBBAAgent agent("robot_1", 5);

    std::vector<CBBAMessage> messages;

    // Should handle empty messages without crashing
    resolver.resolve_conflicts(agent, messages);

    CHECK(agent.get_bundle().empty());
    CHECK(agent.get_path().empty());
}

TEST_CASE("ConsensusResolver - UPDATE Rule - Accept Better Bid") {
    ConsensusResolver resolver;
    CBBAAgent agent1("robot_1", 5);
    agent1.update_pose(consens::Pose(0.0, 0.0, 0.0));
    agent1.update_velocity(2.0);

    // Agent 1 has a bid on task_1
    agent1.add_to_bundle("task_1", 50.0, 0);
    agent1.update_winning_bid("task_1", Bid("robot_1", 50.0, 1.0));
    agent1.update_timestamp("robot_1", 1.0);

    // Create message from robot_2 with better bid
    CBBAMessage msg("robot_2", 2.0);
    msg.winning_bids["task_1"] = Bid("robot_2", 100.0, 2.0); // Better score
    msg.winners["task_1"] = "robot_2";
    msg.timestamps["robot_2"] = 2.0;

    std::vector<CBBAMessage> messages = {msg};

    SUBCASE("Accept better bid and lose task") {
        resolver.resolve_conflicts(agent1, messages);

        // Should update winning bid to robot_2's better bid
        Bid winner_bid = agent1.get_winning_bid("task_1");
        CHECK(winner_bid.agent_id == "robot_2");
        CHECK(winner_bid.score == doctest::Approx(100.0));

        // Should remove task from own bundle (RESET rule)
        CHECK_FALSE(agent1.get_bundle().contains("task_1"));
        CHECK_FALSE(agent1.get_path().contains("task_1"));
    }
}

TEST_CASE("ConsensusResolver - LEAVE Rule - Keep Better Bid") {
    ConsensusResolver resolver;
    CBBAAgent agent1("robot_1", 5);

    // Agent 1 has a better bid on task_1
    agent1.add_to_bundle("task_1", 100.0, 0);
    agent1.update_winning_bid("task_1", Bid("robot_1", 100.0, 1.0));
    agent1.update_timestamp("robot_1", 1.0);

    // Create message from robot_2 with worse bid
    CBBAMessage msg("robot_2", 1.0);
    msg.winning_bids["task_1"] = Bid("robot_2", 50.0, 1.0); // Worse score
    msg.winners["task_1"] = "robot_2";
    msg.timestamps["robot_2"] = 1.0;

    std::vector<CBBAMessage> messages = {msg};

    SUBCASE("Keep own better bid") {
        resolver.resolve_conflicts(agent1, messages);

        // Should keep own better bid
        Bid winner_bid = agent1.get_winning_bid("task_1");
        CHECK(winner_bid.agent_id == "robot_1");
        CHECK(winner_bid.score == doctest::Approx(100.0));

        // Should still have task in bundle
        CHECK(agent1.get_bundle().contains("task_1"));
    }
}

TEST_CASE("ConsensusResolver - RESET Rule - Remove Subsequent Tasks") {
    ConsensusResolver resolver;
    CBBAAgent agent1("robot_1", 5);

    // Agent 1 has 3 tasks in path
    agent1.add_to_bundle("task_1", 50.0, 0);
    agent1.add_to_bundle("task_2", 40.0, 1);
    agent1.add_to_bundle("task_3", 30.0, 2);

    agent1.update_winning_bid("task_1", Bid("robot_1", 50.0, 1.0));
    agent1.update_winning_bid("task_2", Bid("robot_1", 40.0, 1.0));
    agent1.update_winning_bid("task_3", Bid("robot_1", 30.0, 1.0));
    agent1.update_timestamp("robot_1", 1.0);

    // Lose task_2 to robot_2
    CBBAMessage msg("robot_2", 2.0);
    msg.winning_bids["task_2"] = Bid("robot_2", 100.0, 2.0);
    msg.winners["task_2"] = "robot_2";
    msg.timestamps["robot_2"] = 2.0;

    std::vector<CBBAMessage> messages = {msg};

    SUBCASE("Remove lost task and all subsequent tasks") {
        resolver.resolve_conflicts(agent1, messages);

        // Should keep task_1 (before lost task)
        CHECK(agent1.get_bundle().contains("task_1"));

        // Should remove task_2 (lost task) and task_3 (subsequent)
        CHECK_FALSE(agent1.get_bundle().contains("task_2"));
        CHECK_FALSE(agent1.get_bundle().contains("task_3"));

        // Path should only have task_1
        CHECK(agent1.get_path().size() == 1);
        CHECK(agent1.get_path()[0] == "task_1");
    }
}

TEST_CASE("ConsensusResolver - Timestamp Updates") {
    ConsensusResolver resolver;
    CBBAAgent agent1("robot_1", 5);
    agent1.update_timestamp("robot_1", 1.0);

    // Create message from robot_2
    CBBAMessage msg("robot_2", 5.0);
    msg.timestamps["robot_2"] = 5.0;
    msg.timestamps["robot_3"] = 3.0; // Multi-hop info about robot_3

    std::vector<CBBAMessage> messages = {msg};

    SUBCASE("Update timestamps from neighbor") {
        resolver.resolve_conflicts(agent1, messages);

        // Should update timestamp for robot_2
        CHECK(agent1.get_timestamp("robot_2") == doctest::Approx(5.0));

        // Should propagate timestamp for robot_3 (multi-hop)
        CHECK(agent1.get_timestamp("robot_3") == doctest::Approx(3.0));
    }
}

TEST_CASE("ConsensusResolver - Tie Breaking by Agent ID") {
    ConsensusResolver resolver;
    CBBAAgent agent1("robot_2", 5); // Higher ID

    // Agent 2 has a bid
    agent1.add_to_bundle("task_1", 50.0, 0);
    agent1.update_winning_bid("task_1", Bid("robot_2", 50.0, 1.0));
    agent1.update_timestamp("robot_2", 1.0);

    // Message from robot_1 with same score (lower ID wins)
    CBBAMessage msg("robot_1", 1.0);
    msg.winning_bids["task_1"] = Bid("robot_1", 50.0, 1.0); // Same score, lower ID
    msg.winners["task_1"] = "robot_1";
    msg.timestamps["robot_1"] = 1.0;

    std::vector<CBBAMessage> messages = {msg};

    SUBCASE("Lower agent ID wins tie") {
        resolver.resolve_conflicts(agent1, messages);

        // robot_1 (lower ID) should win
        Bid winner_bid = agent1.get_winning_bid("task_1");
        CHECK(winner_bid.agent_id == "robot_1");

        // robot_2 should lose the task
        CHECK_FALSE(agent1.get_bundle().contains("task_1"));
    }
}

TEST_CASE("ConsensusResolver - No Conflict Same Winner") {
    ConsensusResolver resolver;
    CBBAAgent agent1("robot_1", 5);

    // Both agents agree robot_2 is winner
    agent1.update_winning_bid("task_1", Bid("robot_2", 100.0, 1.0));
    agent1.update_timestamp("robot_2", 1.0);

    // Message from robot_3 also knows robot_2 is winner
    CBBAMessage msg("robot_3", 1.0);
    msg.winning_bids["task_1"] = Bid("robot_2", 100.0, 1.0);
    msg.winners["task_1"] = "robot_2";
    msg.timestamps["robot_2"] = 1.0;
    msg.timestamps["robot_3"] = 1.0;

    std::vector<CBBAMessage> messages = {msg};

    SUBCASE("No change when agreeing on winner") {
        resolver.resolve_conflicts(agent1, messages);

        // Winner should still be robot_2
        Bid winner_bid = agent1.get_winning_bid("task_1");
        CHECK(winner_bid.agent_id == "robot_2");
        CHECK(winner_bid.score == doctest::Approx(100.0));
    }
}

TEST_CASE("ConsensusResolver - Multiple Messages") {
    ConsensusResolver resolver;
    CBBAAgent agent1("robot_1", 5);
    agent1.update_timestamp("robot_1", 1.0);

    // Message 1: robot_2 has task_1
    CBBAMessage msg1("robot_2", 2.0);
    msg1.winning_bids["task_1"] = Bid("robot_2", 50.0, 2.0);
    msg1.winners["task_1"] = "robot_2";
    msg1.timestamps["robot_2"] = 2.0;

    // Message 2: robot_3 has task_2
    CBBAMessage msg2("robot_3", 3.0);
    msg2.winning_bids["task_2"] = Bid("robot_3", 60.0, 3.0);
    msg2.winners["task_2"] = "robot_3";
    msg2.timestamps["robot_3"] = 3.0;

    std::vector<CBBAMessage> messages = {msg1, msg2};

    SUBCASE("Process multiple messages") {
        resolver.resolve_conflicts(agent1, messages);

        // Should learn about both tasks
        CHECK(agent1.get_winning_bid("task_1").agent_id == "robot_2");
        CHECK(agent1.get_winning_bid("task_2").agent_id == "robot_3");

        // Should have timestamps for all agents
        CHECK(agent1.get_timestamp("robot_2") == doctest::Approx(2.0));
        CHECK(agent1.get_timestamp("robot_3") == doctest::Approx(3.0));
    }
}

TEST_CASE("ConsensusResolver - Newer Information Wins") {
    ConsensusResolver resolver;
    CBBAAgent agent1("robot_1", 5);

    // Agent has old information
    agent1.update_winning_bid("task_1", Bid("robot_2", 50.0, 1.0));
    agent1.update_timestamp("robot_2", 1.0);

    // Neighbor has newer information (even with worse score)
    CBBAMessage msg("robot_3", 5.0);
    msg.winning_bids["task_1"] = Bid("robot_3", 40.0, 5.0); // Newer timestamp
    msg.winners["task_1"] = "robot_3";
    msg.timestamps["robot_3"] = 5.0;

    std::vector<CBBAMessage> messages = {msg};

    SUBCASE("Newer information overrides older") {
        resolver.resolve_conflicts(agent1, messages);

        // Should accept newer information
        Bid winner_bid = agent1.get_winning_bid("task_1");
        CHECK(winner_bid.agent_id == "robot_3");
        CHECK(winner_bid.timestamp == doctest::Approx(5.0));
    }
}

TEST_CASE("ConsensusResolver - Unassigned Task") {
    ConsensusResolver resolver;
    CBBAAgent agent1("robot_1", 5);

    // Agent has no information about task_1

    // Neighbor knows task_1 is assigned to robot_2
    CBBAMessage msg("robot_2", 2.0);
    msg.winning_bids["task_1"] = Bid("robot_2", 50.0, 2.0);
    msg.winners["task_1"] = "robot_2";
    msg.timestamps["robot_2"] = 2.0;

    std::vector<CBBAMessage> messages = {msg};

    SUBCASE("Learn about new task assignment") {
        resolver.resolve_conflicts(agent1, messages);

        // Should learn about task assignment
        Bid winner_bid = agent1.get_winning_bid("task_1");
        CHECK(winner_bid.agent_id == "robot_2");
        CHECK(winner_bid.score == doctest::Approx(50.0));
    }
}
