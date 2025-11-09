#include "consens/cbba/cbba_agent.hpp"

#include <limits>

namespace consens::cbba {

    CBBAAgent::CBBAAgent(const AgentID &id, size_t capacity)
        : id_(id), velocity_(0.0), bundle_(capacity), converged_(false), bundle_capacity_(capacity) {
        // Initialize own timestamp
        timestamps_[id_] = 0.0;
    }

    void CBBAAgent::update_pose(const Pose &pose) { pose_ = pose; }

    void CBBAAgent::update_velocity(double velocity) { velocity_ = velocity; }

    void CBBAAgent::add_to_bundle(const TaskID &task_id, Score bid, size_t position) {
        // Add to bundle
        bundle_.add(task_id);

        // Insert in path
        if (position == SIZE_MAX) {
            position = path_.size();
        }
        path_.insert(task_id, position);

        // Update winning bid
        update_winning_bid(task_id, Bid(id_, bid, timestamps_[id_]));

        // Store local bid
        local_bids_[task_id] = bid;
    }

    void CBBAAgent::remove_from_bundle(const TaskID &task_id) {
        bundle_.remove(task_id);
        path_.remove(task_id);

        // Note: Don't remove from winning_bids_ or winners_ here
        // Those track global state, not just local bundle
    }

    void CBBAAgent::insert_in_path(const TaskID &task_id, size_t position) { path_.insert(task_id, position); }

    void CBBAAgent::update_winning_bid(const TaskID &task_id, const Bid &bid) {
        winning_bids_[task_id] = bid;
        winners_[task_id] = bid.agent_id;
    }

    void CBBAAgent::reset_task(const TaskID &task_id) {
        // Reset to invalid bid
        winning_bids_[task_id] = Bid::invalid();
        winners_[task_id] = NO_AGENT;

        // Remove from bundle if present
        remove_from_bundle(task_id);

        // Also remove from local bids
        local_bids_.erase(task_id);
    }

    void CBBAAgent::set_local_bid(const TaskID &task_id, Score score) { local_bids_[task_id] = score; }

    Score CBBAAgent::get_local_bid(const TaskID &task_id) const {
        auto it = local_bids_.find(task_id);
        if (it != local_bids_.end()) {
            return it->second;
        }
        return MIN_SCORE;
    }

    void CBBAAgent::update_timestamp(const AgentID &agent_id, Timestamp ts) { timestamps_[agent_id] = ts; }

    Timestamp CBBAAgent::get_timestamp(const AgentID &agent_id) const {
        auto it = timestamps_.find(agent_id);
        if (it != timestamps_.end()) {
            return it->second;
        }
        return 0.0;
    }

    void CBBAAgent::set_own_timestamp(Timestamp ts) { timestamps_[id_] = ts; }

    void CBBAAgent::check_convergence() {
        // Agent has converged if winners haven't changed
        converged_ = (winners_ == previous_winners_);
    }

    void CBBAAgent::save_winners_for_convergence() { previous_winners_ = winners_; }

    Bid CBBAAgent::get_winning_bid(const TaskID &task_id) const {
        auto it = winning_bids_.find(task_id);
        if (it != winning_bids_.end()) {
            return it->second;
        }
        return Bid::invalid();
    }

    AgentID CBBAAgent::get_winner(const TaskID &task_id) const {
        auto it = winners_.find(task_id);
        if (it != winners_.end()) {
            return it->second;
        }
        return NO_AGENT;
    }

} // namespace consens::cbba
