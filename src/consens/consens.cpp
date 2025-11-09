#include "consens/consens.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <stdexcept>

namespace consens {

    // ============================================================================
    // Implementation (Pimpl pattern)
    // ============================================================================

    class Consens::Impl {
      public:
        explicit Impl(const Config &config) : config_(config), iteration_count_(0), converged_(false) {
            if (config_.enable_logging) {
                spdlog::info("[Consens] Initialized for agent: {}", config_.agent_id);
            }
        }

        ~Impl() {
            if (config_.enable_logging) {
                spdlog::info("[Consens] Destroyed for agent: {}", config_.agent_id);
            }
        }

        // State updates
        void update_pose(const Pose &pose) { pose_ = pose; }

        void update_velocity(double velocity) { velocity_ = velocity; }

        void add_task(const Task &task) {
            tasks_[task.get_id()] = task;
            if (config_.enable_logging) {
                spdlog::debug("[Consens][{}] Added task: {}", config_.agent_id, task.get_id());
            }
        }

        void remove_task(const TaskID &id) {
            auto it = tasks_.find(id);
            if (it != tasks_.end()) {
                tasks_.erase(it);

                // Remove from bundle and path if present
                auto bundle_it = std::find(bundle_.begin(), bundle_.end(), id);
                if (bundle_it != bundle_.end()) {
                    bundle_.erase(bundle_it);
                }

                auto path_it = std::find(path_.begin(), path_.end(), id);
                if (path_it != path_.end()) {
                    path_.erase(path_it);
                }

                if (config_.enable_logging) {
                    spdlog::debug("[Consens][{}] Removed task: {}", config_.agent_id, id);
                }
            }
        }

        void mark_task_completed(const TaskID &id) {
            auto it = tasks_.find(id);
            if (it != tasks_.end()) {
                it->second.set_completed(true);

                // Remove from bundle and path
                remove_from_bundle(id);

                if (config_.enable_logging) {
                    spdlog::info("[Consens][{}] Marked task completed: {}", config_.agent_id, id);
                }
            }
        }

        void update_neighbors(const std::vector<AgentID> &neighbor_ids) { neighbors_ = neighbor_ids; }

        void tick(float dt) {
            iteration_count_++;

            if (config_.enable_logging && iteration_count_ % 10 == 0) {
                spdlog::debug("[Consens][{}] Tick {} - {} tasks, {} in bundle", config_.agent_id, iteration_count_,
                              tasks_.size(), bundle_.size());
            }

            // TODO: Implement CBBA algorithm
            // For now, just a dummy implementation that does nothing
        }

        // Query results
        std::vector<TaskID> get_bundle() const { return bundle_; }

        std::vector<TaskID> get_path() const { return path_; }

        std::optional<TaskID> get_next_task() const {
            if (!path_.empty()) {
                return path_[0];
            }
            return std::nullopt;
        }

        std::optional<Task> get_task(const TaskID &id) const {
            auto it = tasks_.find(id);
            if (it != tasks_.end()) {
                return it->second;
            }
            return std::nullopt;
        }

        std::vector<Task> get_all_tasks() const {
            std::vector<Task> result;
            result.reserve(tasks_.size());
            for (const auto &[id, task] : tasks_) {
                result.push_back(task);
            }
            return result;
        }

        bool has_converged() const { return converged_; }

        Consens::Statistics get_statistics() const {
            Statistics stats;
            stats.iteration_count = iteration_count_;
            stats.bundle_size = bundle_.size();
            stats.total_tasks = tasks_.size();
            stats.total_path_score = 0.0; // TODO: Compute from path
            stats.converged = converged_;
            return stats;
        }

        const AgentID &get_agent_id() const { return config_.agent_id; }

        Pose get_pose() const { return pose_; }

        double get_velocity() const { return velocity_; }

        void reset() {
            bundle_.clear();
            path_.clear();
            converged_ = false;
            iteration_count_ = 0;

            if (config_.enable_logging) {
                spdlog::info("[Consens][{}] Reset", config_.agent_id);
            }
        }

      private:
        // Configuration
        Config config_;

        // Agent state
        Pose pose_;
        double velocity_ = 0.0;
        std::vector<AgentID> neighbors_;

        // Tasks
        std::map<TaskID, Task> tasks_;

        // Algorithm state
        std::vector<TaskID> bundle_; // Unordered tasks this agent claims
        std::vector<TaskID> path_;   // Ordered tasks to execute
        size_t iteration_count_;
        bool converged_;

        // Helper methods
        void remove_from_bundle(const TaskID &id) {
            auto bundle_it = std::find(bundle_.begin(), bundle_.end(), id);
            if (bundle_it != bundle_.end()) {
                bundle_.erase(bundle_it);
            }

            auto path_it = std::find(path_.begin(), path_.end(), id);
            if (path_it != path_.end()) {
                path_.erase(path_it);
            }
        }
    };

    // ============================================================================
    // Public Interface Implementation
    // ============================================================================

    Consens::Consens(const Config &config) : impl_(std::make_unique<Impl>(config)) {}

    Consens::~Consens() = default;

    void Consens::update_pose(double x, double y, double heading) { impl_->update_pose(Pose(x, y, heading)); }

    void Consens::update_pose(const Point &position, double heading) { impl_->update_pose(Pose(position, heading)); }

    void Consens::update_pose(const Pose &pose) { impl_->update_pose(pose); }

    void Consens::update_velocity(double velocity) { impl_->update_velocity(velocity); }

    void Consens::add_task(const TaskID &id, const Point &position, double duration) {
        impl_->add_task(Task(id, position, duration));
    }

    void Consens::add_task(const TaskID &id, const Point &head, const Point &tail, double duration) {
        impl_->add_task(Task(id, head, tail, duration));
    }

    void Consens::add_task(const Task &task) { impl_->add_task(task); }

    void Consens::remove_task(const TaskID &id) { impl_->remove_task(id); }

    void Consens::mark_task_completed(const TaskID &id) { impl_->mark_task_completed(id); }

    void Consens::update_neighbors(const std::vector<AgentID> &neighbor_ids) { impl_->update_neighbors(neighbor_ids); }

    void Consens::tick(float dt) { impl_->tick(dt); }

    std::vector<TaskID> Consens::get_bundle() const { return impl_->get_bundle(); }

    std::vector<TaskID> Consens::get_path() const { return impl_->get_path(); }

    std::optional<TaskID> Consens::get_next_task() const { return impl_->get_next_task(); }

    std::optional<Task> Consens::get_task(const TaskID &id) const { return impl_->get_task(id); }

    std::vector<Task> Consens::get_all_tasks() const { return impl_->get_all_tasks(); }

    bool Consens::has_converged() const { return impl_->has_converged(); }

    Consens::Statistics Consens::get_statistics() const { return impl_->get_statistics(); }

    const AgentID &Consens::get_agent_id() const { return impl_->get_agent_id(); }

    Pose Consens::get_pose() const { return impl_->get_pose(); }

    double Consens::get_velocity() const { return impl_->get_velocity(); }

    void Consens::reset() { impl_->reset(); }

} // namespace consens
