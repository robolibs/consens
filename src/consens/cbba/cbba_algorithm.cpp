#include "consens/cbba/cbba_algorithm.hpp"

namespace consens::cbba {

    CBBAAlgorithm::CBBAAlgorithm(const AgentID &agent_id, const CBBAConfig &config, SendCallback send_callback,
                                 ReceiveCallback receive_callback)
        : agent_id_(agent_id), config_(config), send_callback_(send_callback), receive_callback_(receive_callback),
          velocity_(0.0), cbba_agent_(agent_id, config.max_bundle_size), spatial_index_(),
          bundle_builder_(&spatial_index_, config.metric, config.spatial_query_radius, config.bundle_mode),
          consensus_resolver_(), iteration_count_(0), current_time_(0.0) {}

    void CBBAAlgorithm::update_pose(const Pose &pose) {
        pose_ = pose;
        cbba_agent_.update_pose(pose);
    }

    void CBBAAlgorithm::update_velocity(double velocity) {
        velocity_ = velocity;
        cbba_agent_.update_velocity(velocity);
    }

    void CBBAAlgorithm::add_task(const Task &task) {
        tasks_[task.get_id()] = task;
        update_spatial_index();
    }

    void CBBAAlgorithm::remove_task(const TaskID &id) {
        tasks_.erase(id);
        cbba_agent_.remove_from_bundle(id);
        update_spatial_index();
    }

    void CBBAAlgorithm::mark_task_completed(const TaskID &id) {
        auto it = tasks_.find(id);
        if (it != tasks_.end()) {
            it->second.set_completed(true);
            cbba_agent_.remove_from_bundle(id);
        }
    }

    void CBBAAlgorithm::tick(float dt) {
        iteration_count_++;
        current_time_ += dt;

        // Update agent's timestamp
        cbba_agent_.set_own_timestamp(current_time_);

        // Phase 1: Bundle Building
        bundle_building_phase();

        // Phase 2: Communication
        communication_phase();

        // Phase 3: Consensus
        consensus_phase();

        // Check convergence
        cbba_agent_.check_convergence();
    }

    void CBBAAlgorithm::bundle_building_phase() {
        // Get list of available tasks (not completed, not assigned to others with better bid)
        std::vector<TaskID> available_tasks = get_available_tasks();

        // Use bundle builder to select and add tasks
        bundle_builder_.build_bundle(cbba_agent_, available_tasks);
    }

    void CBBAAlgorithm::communication_phase() {
        // Create message with our current state
        CBBAMessage msg = create_message();

        // Serialize and send via callback
        if (send_callback_) {
            std::vector<uint8_t> data = msg.serialize();
            send_callback_(data);
        }
    }

    void CBBAAlgorithm::consensus_phase() {
        // Receive messages from neighbors
        if (receive_callback_) {
            std::vector<std::vector<uint8_t>> raw_messages = receive_callback_();

            // Deserialize messages
            std::vector<CBBAMessage> messages;
            for (const auto &data : raw_messages) {
                CBBAMessage msg;
                if (msg.deserialize(data)) {
                    messages.push_back(msg);
                }
            }

            // Resolve conflicts
            consensus_resolver_.resolve_conflicts(cbba_agent_, messages);
        }
    }

    std::vector<TaskID> CBBAAlgorithm::get_available_tasks() const {
        std::vector<TaskID> available;

        for (const auto &[task_id, task] : tasks_) {
            // Skip completed tasks
            if (task.is_completed()) {
                continue;
            }

            // Skip if already in our bundle
            if (cbba_agent_.get_bundle().contains(task_id)) {
                continue;
            }

            available.push_back(task_id);
        }

        return available;
    }

    CBBAMessage CBBAAlgorithm::create_message() {
        CBBAMessage msg(agent_id_, current_time_);

        // Copy bundle and path from agent
        const auto &bundle_tasks = cbba_agent_.get_bundle().get_tasks();
        for (const auto &task_id : bundle_tasks) {
            msg.bundle.add(task_id);
        }

        const auto &path_tasks = cbba_agent_.get_path().get_tasks();
        for (size_t i = 0; i < path_tasks.size(); ++i) {
            msg.path.insert(path_tasks[i], i);
        }

        // Copy winning bids, winners, and timestamps
        msg.winning_bids = cbba_agent_.get_winning_bids();
        msg.winners = cbba_agent_.get_winners();
        msg.timestamps = cbba_agent_.get_timestamps();

        return msg;
    }

    void CBBAAlgorithm::update_spatial_index() {
        spatial_index_.clear();
        for (const auto &[task_id, task] : tasks_) {
            if (!task.is_completed()) {
                spatial_index_.insert(task);
            }
        }
    }

    std::vector<TaskID> CBBAAlgorithm::get_bundle() const { return cbba_agent_.get_bundle().get_tasks(); }

    std::vector<TaskID> CBBAAlgorithm::get_path() const { return cbba_agent_.get_path().get_tasks(); }

    std::optional<TaskID> CBBAAlgorithm::get_next_task() const {
        const auto &path = cbba_agent_.get_path();
        if (path.empty()) {
            return std::nullopt;
        }
        return path.front();
    }

    std::optional<Task> CBBAAlgorithm::get_task(const TaskID &id) const {
        auto it = tasks_.find(id);
        if (it != tasks_.end()) {
            return it->second;
        }
        return std::nullopt;
    }

    std::vector<Task> CBBAAlgorithm::get_all_tasks() const {
        std::vector<Task> result;
        result.reserve(tasks_.size());
        for (const auto &[id, task] : tasks_) {
            result.push_back(task);
        }
        return result;
    }

    bool CBBAAlgorithm::has_converged() const { return cbba_agent_.has_converged(); }

    void CBBAAlgorithm::reset() {
        cbba_agent_ = CBBAAgent(agent_id_, config_.max_bundle_size);
        iteration_count_ = 0;
        current_time_ = 0.0;
    }

    double CBBAAlgorithm::get_total_score() const {
        // Sum up all winning bids for tasks in our path
        double total_score = 0.0;
        const auto &path = cbba_agent_.get_path();

        for (const auto &task_id : path.get_tasks()) {
            Score bid_score = cbba_agent_.get_local_bid(task_id);
            if (bid_score > MIN_SCORE) {
                total_score += bid_score;
            }
        }

        return total_score;
    }

} // namespace consens::cbba
