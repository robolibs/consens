#pragma once

#include "cbba_agent.hpp"
#include "messages.hpp"
#include "types.hpp"

#include <vector>

namespace consens::cbba {

    /**
     * Consensus resolver implements the conflict resolution phase of CBBA
     * Applies UPDATE, RESET, and LEAVE rules based on neighbor messages
     */
    class ConsensusResolver {
      public:
        ConsensusResolver() = default;
        ~ConsensusResolver() = default;

        /**
         * Resolve conflicts based on neighbor messages
         * Main entry point for consensus phase
         *
         * @param agent Agent whose state to update
         * @param neighbor_messages Messages received from neighboring agents
         */
        void resolve_conflicts(CBBAAgent &agent, const std::vector<CBBAMessage> &neighbor_messages);

      private:
        /**
         * Process a single message from a neighbor
         *
         * @param agent Agent state to update
         * @param msg Message from neighbor
         */
        void process_message(CBBAAgent &agent, const CBBAMessage &msg);

        /**
         * Resolve conflict for a specific task
         * Applies CBBA consensus rules (UPDATE/RESET/LEAVE)
         *
         * @param agent Agent state
         * @param msg Neighbor's message
         * @param task_id Task to resolve conflict for
         */
        void resolve_task_conflict(CBBAAgent &agent, const CBBAMessage &msg, const TaskID &task_id);

        /**
         * UPDATE rule: Accept neighbor's information
         * Called when neighbor has better or newer information
         *
         * @param agent Agent state
         * @param msg Neighbor's message
         * @param task_id Task to update
         */
        void apply_update_rule(CBBAAgent &agent, const CBBAMessage &msg, const TaskID &task_id);

        /**
         * RESET rule: Lost task, remove from bundle
         * Called when agent loses task to another agent
         * Also resets all tasks after this one in path
         *
         * @param agent Agent state
         * @param task_id Task that was lost
         */
        void apply_reset_rule(CBBAAgent &agent, const TaskID &task_id);

        /**
         * LEAVE rule: No conflict, maintain current state
         * Called when no changes needed
         *
         * @param agent Agent state
         */
        void apply_leave_rule(CBBAAgent &agent);

        /**
         * Update timestamps from neighbor's message
         * Implements multi-hop information propagation
         *
         * @param agent Agent state
         * @param msg Neighbor's message
         */
        void update_timestamps(CBBAAgent &agent, const CBBAMessage &msg);

        /**
         * Check if agent i's information about agent k is outdated
         * compared to neighbor j's information
         *
         * @param agent Agent i
         * @param neighbor_id Agent j (neighbor)
         * @param other_agent_id Agent k (third agent)
         * @param neighbor_ts Agent j's timestamp for k
         * @return True if j has newer information about k
         */
        bool has_newer_info(const CBBAAgent &agent, const AgentID &neighbor_id, const AgentID &other_agent_id,
                            Timestamp neighbor_ts) const;
    };

} // namespace consens::cbba
