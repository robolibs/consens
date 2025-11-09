#include "consens/cbba/consensus_resolver.hpp"

#include <algorithm>
#include <set>

namespace consens::cbba {

    void ConsensusResolver::resolve_conflicts(CBBAAgent &agent, const std::vector<CBBAMessage> &neighbor_messages) {
        // Process each neighbor's message
        for (const auto &msg : neighbor_messages) {
            process_message(agent, msg);
        }
    }

    void ConsensusResolver::process_message(CBBAAgent &agent, const CBBAMessage &msg) {
        // First, update timestamps for multi-hop information propagation
        update_timestamps(agent, msg);

        // Get all tasks that either we or the neighbor know about
        // Check conflicts for each task
        std::set<TaskID> all_tasks;

        // Add tasks from our winning bids
        for (const auto &[task_id, bid] : agent.get_winning_bids()) {
            all_tasks.insert(task_id);
        }

        // Add tasks from neighbor's winning bids
        for (const auto &[task_id, bid] : msg.winning_bids) {
            all_tasks.insert(task_id);
        }

        // Resolve conflict for each task
        for (const TaskID &task_id : all_tasks) {
            resolve_task_conflict(agent, msg, task_id);
        }
    }

    void ConsensusResolver::resolve_task_conflict(CBBAAgent &agent, const CBBAMessage &msg, const TaskID &task_id) {
        // Get current information
        Bid my_bid = agent.get_winning_bid(task_id);
        AgentID my_winner = agent.get_winner(task_id);

        // Get neighbor's information
        Bid neighbor_bid = msg.get_winning_bid(task_id);
        AgentID neighbor_winner = msg.get_winner(task_id);

        // CBBA Consensus Rules
        // The key decision: Should we update our information?

        // Case 1: Neighbor has info about a winner we don't know about
        if (neighbor_winner != NO_AGENT && my_winner == NO_AGENT) {
            // UPDATE: Accept neighbor's assignment
            apply_update_rule(agent, msg, task_id);
            return;
        }

        // Case 2: We have info about a winner, neighbor doesn't
        if (my_winner != NO_AGENT && neighbor_winner == NO_AGENT) {
            // LEAVE: Keep our information
            apply_leave_rule(agent);
            return;
        }

        // Case 3: Neither has a winner
        if (my_winner == NO_AGENT && neighbor_winner == NO_AGENT) {
            // LEAVE: Nothing to do
            apply_leave_rule(agent);
            return;
        }

        // Case 4: Both have winners - need to compare information freshness and quality

        // Check if neighbor has newer information about the same winner
        if (my_winner == neighbor_winner) {
            // Same winner - check if neighbor has newer/better info
            // Use bid timestamp to determine freshness
            if (neighbor_bid.timestamp > my_bid.timestamp) {
                // UPDATE: Neighbor has fresher information
                apply_update_rule(agent, msg, task_id);
                return;
            } else {
                // LEAVE: Our information is up to date
                apply_leave_rule(agent);
                return;
            }
        }

        // Different winners - need to determine who should win
        // Compare based on bid timestamp and quality

        // If one bid has newer timestamp, use that
        if (neighbor_bid.timestamp > my_bid.timestamp) {
            // Neighbor has newer info - UPDATE
            apply_update_rule(agent, msg, task_id);

            // RESET: If we lost this task, remove it from our bundle
            if (my_winner == agent.get_id() && neighbor_winner != agent.get_id()) {
                apply_reset_rule(agent, task_id);
            }
            return;
        } else if (my_bid.timestamp > neighbor_bid.timestamp) {
            // Our info is newer - LEAVE
            apply_leave_rule(agent);
            return;
        }

        // Same timestamp - compare bids by score (and tie-break by agent ID)
        if (neighbor_bid > my_bid) {
            // Neighbor has better bid - UPDATE
            apply_update_rule(agent, msg, task_id);

            // RESET: If we lost this task, remove it from our bundle
            if (my_winner == agent.get_id() && neighbor_winner != agent.get_id()) {
                apply_reset_rule(agent, task_id);
            }
            return;
        } else {
            // Our bid is better or equal - LEAVE
            apply_leave_rule(agent);
            return;
        }
    }

    void ConsensusResolver::apply_update_rule(CBBAAgent &agent, const CBBAMessage &msg, const TaskID &task_id) {
        // Update our winning bid and winner with neighbor's information
        Bid neighbor_bid = msg.get_winning_bid(task_id);
        agent.update_winning_bid(task_id, neighbor_bid);
    }

    void ConsensusResolver::apply_reset_rule(CBBAAgent &agent, const TaskID &task_id) {
        // Lost this task - remove from bundle and path

        // Find position in path
        const Path &path = agent.get_path();
        size_t position = path.find_position(task_id);

        if (position < path.size()) {
            // Remove this task and all subsequent tasks from bundle and path
            // This is because subsequent tasks depend on completing this one first

            // Get all tasks from this position onwards
            std::vector<TaskID> tasks_to_remove;
            for (size_t i = position; i < path.size(); ++i) {
                tasks_to_remove.push_back(path[i]);
            }

            // Remove each task from bundle/path
            // NOTE: We DON'T call reset_task because we want to keep the winning bid information
            // that was set by apply_update_rule (the neighbor's better bid)
            for (const TaskID &tid : tasks_to_remove) {
                agent.remove_from_bundle(tid);
            }
        }
    }

    void ConsensusResolver::apply_leave_rule(CBBAAgent &agent) {
        // No-op: maintain current state
        // This is just for clarity in the algorithm structure
        (void)agent; // Suppress unused parameter warning
    }

    void ConsensusResolver::update_timestamps(CBBAAgent &agent, const CBBAMessage &msg) {
        // Update timestamp for the sender
        agent.update_timestamp(msg.sender_id, msg.timestamp);

        // Multi-hop: propagate timestamps from neighbor's knowledge
        // This allows information to spread beyond direct neighbors
        for (const auto &[other_agent_id, neighbor_ts] : msg.timestamps) {
            // Check if neighbor has newer information about other_agent_id
            if (has_newer_info(agent, msg.sender_id, other_agent_id, neighbor_ts)) {
                // Update our timestamp for other_agent_id
                agent.update_timestamp(other_agent_id, neighbor_ts);
            }
        }
    }

    bool ConsensusResolver::has_newer_info(const CBBAAgent &agent, const AgentID &neighbor_id,
                                           const AgentID &other_agent_id, Timestamp neighbor_ts) const {
        // Get our current timestamp for other_agent_id
        Timestamp my_ts = agent.get_timestamp(other_agent_id);

        // Neighbor has newer info if:
        // 1. Their timestamp for other_agent is newer than ours, OR
        // 2. We have no information (timestamp = 0) and they do

        return neighbor_ts > my_ts;
    }

} // namespace consens::cbba
