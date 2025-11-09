#pragma once

#include "types.hpp"

#include <map>

namespace consens::cbba {

    /**
     * Bid structure representing an agent's bid on a task
     */
    struct Bid {
        AgentID agent_id;
        Score score;
        Timestamp timestamp;

        Bid() : agent_id(NO_AGENT), score(MIN_SCORE), timestamp(0.0) {}

        Bid(const AgentID &id, Score s, Timestamp t) : agent_id(id), score(s), timestamp(t) {}

        /**
         * Comparison for auction logic
         * Higher score wins; if scores equal, lower agent ID wins
         */
        bool operator>(const Bid &other) const {
            if (agent_id == NO_AGENT && other.agent_id != NO_AGENT) {
                return false; // Unassigned loses to assigned
            }
            if (agent_id != NO_AGENT && other.agent_id == NO_AGENT) {
                return true; // Assigned beats unassigned
            }

            if (score > other.score) {
                return true;
            } else if (score == other.score) {
                // Tie-breaking: lower agent ID wins
                return agent_id < other.agent_id;
            }
            return false;
        }

        bool operator==(const Bid &other) const {
            return agent_id == other.agent_id && score == other.score && timestamp == other.timestamp;
        }

        bool operator!=(const Bid &other) const { return !(*this == other); }

        /**
         * Check if bid is valid (assigned to an agent)
         */
        bool is_valid() const { return agent_id != NO_AGENT && score > MIN_SCORE; }

        /**
         * Create an invalid/unassigned bid
         */
        static Bid invalid() { return Bid(); }
    };

    /**
     * Winning bids for each task
     * Maps TaskID -> Bid
     */
    using TaskBids = std::map<TaskID, Bid>;

    /**
     * Winners for each task (just the agent ID)
     * Maps TaskID -> AgentID
     */
    using TaskWinners = std::map<TaskID, AgentID>;

    /**
     * Agent timestamps (for consensus protocol)
     * Maps AgentID -> Timestamp
     */
    using AgentTimestamps = std::map<AgentID, Timestamp>;

} // namespace consens::cbba
