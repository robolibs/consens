#pragma once

#include "../types.hpp"
#include "bid.hpp"
#include "bundle.hpp"
#include "types.hpp"

namespace consens::cbba {

    /**
     * CBBA agent state
     * Stores all internal state for the CBBA algorithm for one agent
     * This is the core of the CBBA algorithm implementation
     */
    class CBBAAgent {
      private:
        // Agent identification
        AgentID id_;

        // Current agent state (updated from simulator)
        Pose pose_;
        double velocity_;

        // CBBA state vectors
        Bundle bundle_;                      // b: bundle (unordered tasks this agent claims)
        Path path_;                          // p: path (ordered tasks to execute)
        TaskBids winning_bids_;              // y: winning bid for each task
        TaskWinners winners_;                // z: winning agent for each task
        std::map<TaskID, Score> local_bids_; // c: my computed bids (marginal gains)
        AgentTimestamps timestamps_;         // s: timestamps for each agent (for consensus)

        // Convergence tracking
        bool converged_;
        TaskWinners previous_winners_; // For detecting convergence

        // Configuration
        size_t bundle_capacity_;

      public:
        /**
         * Constructor
         */
        CBBAAgent(const AgentID &id, size_t capacity);

        // ========== State Updates ==========

        /**
         * Update agent pose from simulator
         */
        void update_pose(const Pose &pose);

        /**
         * Update agent velocity from simulator
         */
        void update_velocity(double velocity);

        // ========== Bundle Operations ==========

        /**
         * Add a task to bundle and path
         * @param task_id Task to add
         * @param bid Bid value for this task
         * @param position Position in path to insert (default: end)
         */
        void add_to_bundle(const TaskID &task_id, Score bid, size_t position = SIZE_MAX);

        /**
         * Remove a task from bundle and path
         */
        void remove_from_bundle(const TaskID &task_id);

        /**
         * Insert task in path at specific position
         */
        void insert_in_path(const TaskID &task_id, size_t position);

        // ========== Bid Management ==========

        /**
         * Update winning bid for a task
         */
        void update_winning_bid(const TaskID &task_id, const Bid &bid);

        /**
         * Reset a task (mark as unassigned)
         * Used in consensus when task is lost
         */
        void reset_task(const TaskID &task_id);

        /**
         * Set local bid (computed marginal gain) for a task
         */
        void set_local_bid(const TaskID &task_id, Score score);

        /**
         * Get local bid for a task
         */
        Score get_local_bid(const TaskID &task_id) const;

        // ========== Timestamp Management ==========

        /**
         * Update timestamp for an agent (consensus protocol)
         */
        void update_timestamp(const AgentID &agent_id, Timestamp ts);

        /**
         * Get timestamp for an agent
         */
        Timestamp get_timestamp(const AgentID &agent_id) const;

        /**
         * Set own timestamp
         */
        void set_own_timestamp(Timestamp ts);

        // ========== Convergence ==========

        /**
         * Check if agent has converged
         * (winners haven't changed since last check)
         */
        void check_convergence();

        /**
         * Save current winners for convergence detection
         */
        void save_winners_for_convergence();

        // ========== Getters ==========

        const AgentID &get_id() const { return id_; }
        const Pose &get_pose() const { return pose_; }
        double get_velocity() const { return velocity_; }

        const Bundle &get_bundle() const { return bundle_; }
        Bundle &get_bundle() { return bundle_; }

        const Path &get_path() const { return path_; }
        Path &get_path() { return path_; }

        const TaskBids &get_winning_bids() const { return winning_bids_; }
        TaskBids &get_winning_bids() { return winning_bids_; }

        const TaskWinners &get_winners() const { return winners_; }
        TaskWinners &get_winners() { return winners_; }

        const AgentTimestamps &get_timestamps() const { return timestamps_; }
        AgentTimestamps &get_timestamps() { return timestamps_; }

        bool has_converged() const { return converged_; }

        /**
         * Get winning bid for a specific task
         */
        Bid get_winning_bid(const TaskID &task_id) const;

        /**
         * Get winner for a specific task
         */
        AgentID get_winner(const TaskID &task_id) const;
    };

} // namespace consens::cbba
