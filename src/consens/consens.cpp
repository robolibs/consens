#include "consens/consens.hpp"

#include "consens/algorithm.hpp"
#include "consens/cbba/cbba_algorithm.hpp"
#include "consens/cbba/types.hpp"

#include <spdlog/spdlog.h>

#include <stdexcept>

namespace consens {

    // ============================================================================
    // Implementation (Pimpl pattern with Strategy pattern for algorithm)
    // ============================================================================

    class Consens::Impl {
      public:
        explicit Impl(const Config &config) : config_(config) {
            // Default: create CBBA algorithm
            cbba::CBBAConfig cbba_config;
            cbba_config.max_bundle_size = config.max_bundle_size;
            cbba_config.spatial_query_radius = config.spatial_query_radius;

            auto cbba_alg =
                new cbba::CBBAAlgorithm(config.agent_id, cbba_config, config.send_message, config.receive_messages);
            algorithm_.reset(static_cast<Algorithm *>(cbba_alg));

            if (config_.enable_logging) {
                spdlog::info("[Consens] Initialized agent: {} with CBBA algorithm", config_.agent_id);
            }
        }

        explicit Impl(const Config &config, std::unique_ptr<Algorithm> algorithm)
            : config_(config), algorithm_(std::move(algorithm)) {
            if (!algorithm_) {
                throw std::invalid_argument("Algorithm cannot be null");
            }

            if (config_.enable_logging) {
                spdlog::info("[Consens] Initialized agent: {} with custom algorithm", config_.agent_id);
            }
        }

        ~Impl() {
            if (config_.enable_logging) {
                spdlog::info("[Consens] Destroyed agent: {}", config_.agent_id);
            }
        }

        // State updates - delegate to algorithm
        void update_pose(const Pose &pose) {
            if (algorithm_) {
                algorithm_->update_pose(pose);
            }
        }

        void update_velocity(double velocity) {
            if (algorithm_) {
                algorithm_->update_velocity(velocity);
            }
        }

        void add_task(const Task &task) {
            if (algorithm_) {
                algorithm_->add_task(task);
            }
        }

        void remove_task(const TaskID &id) {
            if (algorithm_) {
                algorithm_->remove_task(id);
            }
        }

        void mark_task_completed(const TaskID &id) {
            if (algorithm_) {
                algorithm_->mark_task_completed(id);
            }
        }

        void update_neighbors(const std::vector<AgentID> &neighbor_ids) {
            // Store for potential future use
            neighbors_ = neighbor_ids;
        }

        void tick(float dt) {
            if (algorithm_) {
                algorithm_->tick(dt);
            }
        }

        // Query results - delegate to algorithm
        std::vector<TaskID> get_bundle() const {
            if (algorithm_) {
                return algorithm_->get_bundle();
            }
            return {};
        }

        std::vector<TaskID> get_path() const {
            if (algorithm_) {
                return algorithm_->get_path();
            }
            return {};
        }

        std::optional<TaskID> get_next_task() const {
            if (algorithm_) {
                return algorithm_->get_next_task();
            }
            return std::nullopt;
        }

        std::optional<Task> get_task(const TaskID &id) const {
            if (algorithm_) {
                return algorithm_->get_task(id);
            }
            return std::nullopt;
        }

        std::vector<Task> get_all_tasks() const {
            if (algorithm_) {
                return algorithm_->get_all_tasks();
            }
            return {};
        }

        bool has_converged() const {
            if (algorithm_) {
                return algorithm_->has_converged();
            }
            return false;
        }

        Consens::Statistics get_statistics() const {
            Statistics stats;

            if (algorithm_) {
                stats.bundle_size = algorithm_->get_bundle().size();
                stats.total_tasks = algorithm_->get_all_tasks().size();
                stats.total_path_score = algorithm_->get_total_score();
                stats.converged = algorithm_->has_converged();
            } else {
                stats.bundle_size = 0;
                stats.total_tasks = 0;
                stats.total_path_score = 0.0;
                stats.converged = false;
            }

            stats.iteration_count = iteration_count_;
            return stats;
        }

        const AgentID &get_agent_id() const { return config_.agent_id; }

        Pose get_pose() const {
            // Algorithm doesn't expose pose, so we'd need to track it
            // For now return default
            return Pose();
        }

        double get_velocity() const {
            // Algorithm doesn't expose velocity, so we'd need to track it
            // For now return default
            return 0.0;
        }

        void reset() {
            if (algorithm_) {
                algorithm_->reset();
            }
            iteration_count_ = 0;

            if (config_.enable_logging) {
                spdlog::info("[Consens] Reset agent: {}", config_.agent_id);
            }
        }

      private:
        // Configuration
        Config config_;

        // Algorithm (strategy pattern)
        std::unique_ptr<Algorithm> algorithm_;

        // State tracking
        std::vector<AgentID> neighbors_;
        size_t iteration_count_ = 0;
    };

    // ============================================================================
    // Public Interface Implementation
    // ============================================================================

    Consens::Consens(const Config &config) : impl_(std::make_unique<Impl>(config)) {}

    Consens::Consens(const Config &config, std::unique_ptr<Algorithm> algorithm)
        : impl_(std::make_unique<Impl>(config, std::move(algorithm))) {}

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
