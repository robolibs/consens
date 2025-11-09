#pragma once

#include "bid.hpp"
#include "bundle.hpp"
#include "types.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace consens::cbba {

    /**
     * CBBA message structure for inter-agent communication
     * Contains all information needed for consensus resolution
     */
    struct CBBAMessage {
        // Message metadata
        AgentID sender_id;   // Agent sending this message
        Timestamp timestamp; // When message was created

        // Agent's current assignment
        Bundle bundle; // Tasks claimed by sender
        Path path;     // Ordered execution path

        // Winning bid information (shared across all agents)
        TaskBids winning_bids; // Best bid for each task (y vector)
        TaskWinners winners;   // Winner agent for each task (z vector)

        // Timestamps for multi-hop consensus
        AgentTimestamps timestamps; // Sender's knowledge of other agents' timestamps (s vector)

        /**
         * Default constructor
         */
        CBBAMessage() : sender_id(NO_AGENT), timestamp(0.0) {}

        /**
         * Constructor with sender info
         */
        CBBAMessage(const AgentID &sender, Timestamp ts) : sender_id(sender), timestamp(ts) {}

        /**
         * Serialize message to binary format for transmission
         * Returns byte vector suitable for network transmission
         */
        std::vector<uint8_t> serialize() const;

        /**
         * Deserialize message from binary format
         * Returns true if successful, false if data is invalid
         */
        bool deserialize(const std::vector<uint8_t> &data);

        /**
         * Get winning bid for a specific task
         */
        Bid get_winning_bid(const TaskID &task_id) const {
            auto it = winning_bids.find(task_id);
            if (it != winning_bids.end()) {
                return it->second;
            }
            return Bid::invalid();
        }

        /**
         * Get winner for a specific task
         */
        AgentID get_winner(const TaskID &task_id) const {
            auto it = winners.find(task_id);
            if (it != winners.end()) {
                return it->second;
            }
            return NO_AGENT;
        }

        /**
         * Get timestamp for a specific agent
         */
        Timestamp get_timestamp(const AgentID &agent_id) const {
            auto it = timestamps.find(agent_id);
            if (it != timestamps.end()) {
                return it->second;
            }
            return 0.0;
        }
    };

} // namespace consens::cbba
