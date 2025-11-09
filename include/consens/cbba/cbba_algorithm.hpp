#pragma once

#include "../algorithm.hpp"
#include "../types.hpp"
#include "bundle_builder.hpp"
#include "cbba_agent.hpp"
#include "consensus_resolver.hpp"
#include "messages.hpp"
#include "spatial_index.hpp"
#include "types.hpp"

#include <map>
#include <memory>

namespace consens::cbba {

    /**
     * CBBA (Consensus-Based Bundle Algorithm) implementation
     * Distributed task allocation algorithm for multi-agent systems
     */
    class CBBAAlgorithm : public Algorithm {
      public:
        /**
         * Constructor
         */
        CBBAAlgorithm(const AgentID &agent_id, const CBBAConfig &config, SendCallback send_callback,
                      ReceiveCallback receive_callback);

        ~CBBAAlgorithm() override = default;

        // Implement Algorithm interface
        void update_pose(const Pose &pose) override;
        void update_velocity(double velocity) override;
        void add_task(const Task &task) override;
        void remove_task(const TaskID &id) override;
        void mark_task_completed(const TaskID &id) override;
        void tick(float dt) override;
        std::vector<TaskID> get_bundle() const override;
        std::vector<TaskID> get_path() const override;
        std::optional<TaskID> get_next_task() const override;
        std::optional<Task> get_task(const TaskID &id) const override;
        std::vector<Task> get_all_tasks() const override;
        bool has_converged() const override;
        void reset() override;
        double get_total_score() const override;

      private:
        // Configuration
        AgentID agent_id_;
        CBBAConfig config_;
        SendCallback send_callback_;
        ReceiveCallback receive_callback_;

        // Agent state
        Pose pose_;
        double velocity_;

        // CBBA components
        CBBAAgent cbba_agent_;
        SpatialIndex spatial_index_;
        BundleBuilder bundle_builder_;
        ConsensusResolver consensus_resolver_;

        // Tasks
        std::map<TaskID, Task> tasks_;

        // State
        size_t iteration_count_;
        double current_time_;

        // CBBA phases
        void bundle_building_phase();
        void communication_phase();
        void consensus_phase();

        // Helper methods
        std::vector<TaskID> get_available_tasks() const;
        CBBAMessage create_message();
        void update_spatial_index();
    };

} // namespace consens::cbba
