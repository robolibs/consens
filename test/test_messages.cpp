#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <consens/cbba/messages.hpp>

using namespace consens::cbba;

TEST_CASE("CBBAMessage - Empty Message Serialization") {
    CBBAMessage msg;

    // Serialize
    std::vector<uint8_t> data = msg.serialize();

    // Should have some data
    CHECK(data.size() > 0);

    // Deserialize
    CBBAMessage msg2;
    bool success = msg2.deserialize(data);

    CHECK(success);
    CHECK(msg2.sender_id == msg.sender_id);
    CHECK(msg2.timestamp == doctest::Approx(msg.timestamp));
}

TEST_CASE("CBBAMessage - Basic Message Serialization") {
    CBBAMessage msg("robot_1", 10.5);

    // Serialize
    std::vector<uint8_t> data = msg.serialize();

    // Deserialize
    CBBAMessage msg2;
    bool success = msg2.deserialize(data);

    CHECK(success);
    CHECK(msg2.sender_id == "robot_1");
    CHECK(msg2.timestamp == doctest::Approx(10.5));
}

TEST_CASE("CBBAMessage - Bundle Serialization") {
    CBBAMessage msg("robot_1", 5.0);

    // Add tasks to bundle
    msg.bundle.add("task_1");
    msg.bundle.add("task_2");
    msg.bundle.add("task_3");

    // Serialize
    std::vector<uint8_t> data = msg.serialize();

    // Deserialize
    CBBAMessage msg2;
    bool success = msg2.deserialize(data);

    CHECK(success);
    CHECK(msg2.bundle.size() == 3);
    CHECK(msg2.bundle.contains("task_1"));
    CHECK(msg2.bundle.contains("task_2"));
    CHECK(msg2.bundle.contains("task_3"));
}

TEST_CASE("CBBAMessage - Path Serialization") {
    CBBAMessage msg("robot_1", 5.0);

    // Add tasks to path
    msg.path.insert("task_1", 0);
    msg.path.insert("task_2", 1);
    msg.path.insert("task_3", 2);

    // Serialize
    std::vector<uint8_t> data = msg.serialize();

    // Deserialize
    CBBAMessage msg2;
    bool success = msg2.deserialize(data);

    CHECK(success);
    CHECK(msg2.path.size() == 3);
    CHECK(msg2.path[0] == "task_1");
    CHECK(msg2.path[1] == "task_2");
    CHECK(msg2.path[2] == "task_3");
}

TEST_CASE("CBBAMessage - Winning Bids Serialization") {
    CBBAMessage msg("robot_1", 5.0);

    // Add winning bids
    msg.winning_bids["task_1"] = Bid("robot_1", 50.0, 1.0);
    msg.winning_bids["task_2"] = Bid("robot_2", 75.5, 2.5);
    msg.winning_bids["task_3"] = Bid("robot_1", 30.2, 3.7);

    // Serialize
    std::vector<uint8_t> data = msg.serialize();

    // Deserialize
    CBBAMessage msg2;
    bool success = msg2.deserialize(data);

    CHECK(success);
    CHECK(msg2.winning_bids.size() == 3);

    // Check task_1
    auto bid1 = msg2.get_winning_bid("task_1");
    CHECK(bid1.agent_id == "robot_1");
    CHECK(bid1.score == doctest::Approx(50.0));
    CHECK(bid1.timestamp == doctest::Approx(1.0));

    // Check task_2
    auto bid2 = msg2.get_winning_bid("task_2");
    CHECK(bid2.agent_id == "robot_2");
    CHECK(bid2.score == doctest::Approx(75.5));
    CHECK(bid2.timestamp == doctest::Approx(2.5));

    // Check task_3
    auto bid3 = msg2.get_winning_bid("task_3");
    CHECK(bid3.agent_id == "robot_1");
    CHECK(bid3.score == doctest::Approx(30.2));
    CHECK(bid3.timestamp == doctest::Approx(3.7));
}

TEST_CASE("CBBAMessage - Winners Serialization") {
    CBBAMessage msg("robot_1", 5.0);

    // Add winners
    msg.winners["task_1"] = "robot_1";
    msg.winners["task_2"] = "robot_2";
    msg.winners["task_3"] = "robot_3";

    // Serialize
    std::vector<uint8_t> data = msg.serialize();

    // Deserialize
    CBBAMessage msg2;
    bool success = msg2.deserialize(data);

    CHECK(success);
    CHECK(msg2.winners.size() == 3);
    CHECK(msg2.get_winner("task_1") == "robot_1");
    CHECK(msg2.get_winner("task_2") == "robot_2");
    CHECK(msg2.get_winner("task_3") == "robot_3");
}

TEST_CASE("CBBAMessage - Agent Timestamps Serialization") {
    CBBAMessage msg("robot_1", 5.0);

    // Add agent timestamps
    msg.timestamps["robot_1"] = 10.5;
    msg.timestamps["robot_2"] = 8.3;
    msg.timestamps["robot_3"] = 12.7;

    // Serialize
    std::vector<uint8_t> data = msg.serialize();

    // Deserialize
    CBBAMessage msg2;
    bool success = msg2.deserialize(data);

    CHECK(success);
    CHECK(msg2.timestamps.size() == 3);
    CHECK(msg2.get_timestamp("robot_1") == doctest::Approx(10.5));
    CHECK(msg2.get_timestamp("robot_2") == doctest::Approx(8.3));
    CHECK(msg2.get_timestamp("robot_3") == doctest::Approx(12.7));
}

TEST_CASE("CBBAMessage - Complete Message Serialization") {
    CBBAMessage msg("robot_1", 15.5);

    // Bundle
    msg.bundle.add("task_1");
    msg.bundle.add("task_2");

    // Path
    msg.path.insert("task_1", 0);
    msg.path.insert("task_2", 1);

    // Winning bids
    msg.winning_bids["task_1"] = Bid("robot_1", 50.0, 1.0);
    msg.winning_bids["task_2"] = Bid("robot_2", 75.0, 2.0);
    msg.winning_bids["task_3"] = Bid("robot_3", 60.0, 3.0);

    // Winners
    msg.winners["task_1"] = "robot_1";
    msg.winners["task_2"] = "robot_2";
    msg.winners["task_3"] = "robot_3";

    // Timestamps
    msg.timestamps["robot_1"] = 15.5;
    msg.timestamps["robot_2"] = 12.0;
    msg.timestamps["robot_3"] = 10.0;

    // Serialize
    std::vector<uint8_t> data = msg.serialize();

    // Deserialize
    CBBAMessage msg2;
    bool success = msg2.deserialize(data);

    CHECK(success);

    // Verify all fields
    CHECK(msg2.sender_id == "robot_1");
    CHECK(msg2.timestamp == doctest::Approx(15.5));

    CHECK(msg2.bundle.size() == 2);
    CHECK(msg2.bundle.contains("task_1"));
    CHECK(msg2.bundle.contains("task_2"));

    CHECK(msg2.path.size() == 2);
    CHECK(msg2.path[0] == "task_1");
    CHECK(msg2.path[1] == "task_2");

    CHECK(msg2.winning_bids.size() == 3);
    CHECK(msg2.get_winning_bid("task_1").agent_id == "robot_1");
    CHECK(msg2.get_winning_bid("task_2").agent_id == "robot_2");
    CHECK(msg2.get_winning_bid("task_3").agent_id == "robot_3");

    CHECK(msg2.winners.size() == 3);
    CHECK(msg2.get_winner("task_1") == "robot_1");
    CHECK(msg2.get_winner("task_2") == "robot_2");
    CHECK(msg2.get_winner("task_3") == "robot_3");

    CHECK(msg2.timestamps.size() == 3);
    CHECK(msg2.get_timestamp("robot_1") == doctest::Approx(15.5));
    CHECK(msg2.get_timestamp("robot_2") == doctest::Approx(12.0));
    CHECK(msg2.get_timestamp("robot_3") == doctest::Approx(10.0));
}

TEST_CASE("CBBAMessage - Invalid Data Deserialization") {
    CBBAMessage msg;

    SUBCASE("Empty data") {
        std::vector<uint8_t> empty_data;
        bool success = msg.deserialize(empty_data);
        CHECK_FALSE(success);
    }

    SUBCASE("Truncated data") {
        // Create valid message
        CBBAMessage valid_msg("robot_1", 5.0);
        valid_msg.bundle.add("task_1");
        std::vector<uint8_t> data = valid_msg.serialize();

        // Truncate it
        data.resize(data.size() / 2);

        bool success = msg.deserialize(data);
        CHECK_FALSE(success);
    }
}

TEST_CASE("CBBAMessage - Round Trip With Special Characters") {
    CBBAMessage msg("robot_with_underscore_123", 42.0);

    // Add tasks with various characters
    msg.bundle.add("task_alpha");
    msg.bundle.add("task_beta_123");
    msg.bundle.add("task-gamma");

    msg.winning_bids["task_alpha"] = Bid("robot_1", 10.0, 1.0);
    msg.winners["task_alpha"] = "robot_1";
    msg.timestamps["robot_with_underscore_123"] = 42.0;

    // Serialize and deserialize
    std::vector<uint8_t> data = msg.serialize();
    CBBAMessage msg2;
    bool success = msg2.deserialize(data);

    CHECK(success);
    CHECK(msg2.sender_id == "robot_with_underscore_123");
    CHECK(msg2.bundle.contains("task_alpha"));
    CHECK(msg2.bundle.contains("task_beta_123"));
    CHECK(msg2.bundle.contains("task-gamma"));
}

TEST_CASE("CBBAMessage - Large Message") {
    CBBAMessage msg("robot_1", 100.0);

    // Bundle now has unlimited capacity by default - no need to set it
    // Add many tasks
    for (int i = 0; i < 50; ++i) {
        std::string task_id = "task_" + std::to_string(i);
        msg.bundle.add(task_id);
        msg.winning_bids[task_id] = Bid("robot_1", i * 10.0, i * 1.0);
        msg.winners[task_id] = "robot_1";
    }

    // Add many agents
    for (int i = 0; i < 20; ++i) {
        std::string agent_id = "robot_" + std::to_string(i);
        msg.timestamps[agent_id] = i * 5.0;
    }

    // Serialize
    std::vector<uint8_t> data = msg.serialize();

    // Should have substantial size
    CHECK(data.size() > 1000);

    // Deserialize
    CBBAMessage msg2;
    bool success = msg2.deserialize(data);

    CHECK(success);
    CHECK(msg2.bundle.size() == 50);
    CHECK(msg2.winning_bids.size() == 50);
    CHECK(msg2.timestamps.size() == 20);

    // Spot check some values
    CHECK(msg2.bundle.contains("task_0"));
    CHECK(msg2.bundle.contains("task_49"));
    CHECK(msg2.get_winning_bid("task_10").score == doctest::Approx(100.0));
    CHECK(msg2.get_timestamp("robot_5") == doctest::Approx(25.0));
}
