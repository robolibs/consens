#include "consens/cbba/messages.hpp"

#include <cstring>

namespace consens::cbba {

    // Helper class for binary serialization
    class BinaryWriter {
      private:
        std::vector<uint8_t> buffer_;

      public:
        // Write primitive types
        void write_double(double value) {
            const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&value);
            buffer_.insert(buffer_.end(), bytes, bytes + sizeof(double));
        }

        void write_uint32(uint32_t value) {
            const uint8_t *bytes = reinterpret_cast<const uint8_t *>(&value);
            buffer_.insert(buffer_.end(), bytes, bytes + sizeof(uint32_t));
        }

        void write_string(const std::string &str) {
            // Write length first
            write_uint32(static_cast<uint32_t>(str.size()));
            // Write string data
            buffer_.insert(buffer_.end(), str.begin(), str.end());
        }

        void write_task_ids(const std::vector<TaskID> &tasks) {
            write_uint32(static_cast<uint32_t>(tasks.size()));
            for (const auto &task_id : tasks) {
                write_string(task_id);
            }
        }

        void write_bid(const Bid &bid) {
            write_string(bid.agent_id);
            write_double(bid.score);
            write_double(bid.timestamp);
        }

        void write_task_bids(const TaskBids &bids) {
            write_uint32(static_cast<uint32_t>(bids.size()));
            for (const auto &[task_id, bid] : bids) {
                write_string(task_id);
                write_bid(bid);
            }
        }

        void write_task_winners(const TaskWinners &winners) {
            write_uint32(static_cast<uint32_t>(winners.size()));
            for (const auto &[task_id, agent_id] : winners) {
                write_string(task_id);
                write_string(agent_id);
            }
        }

        void write_agent_timestamps(const AgentTimestamps &timestamps) {
            write_uint32(static_cast<uint32_t>(timestamps.size()));
            for (const auto &[agent_id, ts] : timestamps) {
                write_string(agent_id);
                write_double(ts);
            }
        }

        const std::vector<uint8_t> &get_buffer() const { return buffer_; }
    };

    // Helper class for binary deserialization
    class BinaryReader {
      private:
        const uint8_t *data_;
        size_t size_;
        size_t pos_;

      public:
        BinaryReader(const std::vector<uint8_t> &data) : data_(data.data()), size_(data.size()), pos_(0) {}

        bool has_data(size_t bytes) const { return pos_ + bytes <= size_; }

        bool read_double(double &value) {
            if (!has_data(sizeof(double))) return false;
            std::memcpy(&value, data_ + pos_, sizeof(double));
            pos_ += sizeof(double);
            return true;
        }

        bool read_uint32(uint32_t &value) {
            if (!has_data(sizeof(uint32_t))) return false;
            std::memcpy(&value, data_ + pos_, sizeof(uint32_t));
            pos_ += sizeof(uint32_t);
            return true;
        }

        bool read_string(std::string &str) {
            uint32_t length;
            if (!read_uint32(length)) return false;
            if (!has_data(length)) return false;
            str.assign(reinterpret_cast<const char *>(data_ + pos_), length);
            pos_ += length;
            return true;
        }

        bool read_task_ids(std::vector<TaskID> &tasks) {
            uint32_t count;
            if (!read_uint32(count)) return false;
            tasks.clear();
            tasks.reserve(count);
            for (uint32_t i = 0; i < count; ++i) {
                std::string task_id;
                if (!read_string(task_id)) return false;
                tasks.push_back(task_id);
            }
            return true;
        }

        bool read_bid(Bid &bid) {
            if (!read_string(bid.agent_id)) return false;
            if (!read_double(bid.score)) return false;
            if (!read_double(bid.timestamp)) return false;
            return true;
        }

        bool read_task_bids(TaskBids &bids) {
            uint32_t count;
            if (!read_uint32(count)) return false;
            bids.clear();
            for (uint32_t i = 0; i < count; ++i) {
                std::string task_id;
                Bid bid;
                if (!read_string(task_id)) return false;
                if (!read_bid(bid)) return false;
                bids[task_id] = bid;
            }
            return true;
        }

        bool read_task_winners(TaskWinners &winners) {
            uint32_t count;
            if (!read_uint32(count)) return false;
            winners.clear();
            for (uint32_t i = 0; i < count; ++i) {
                std::string task_id;
                std::string agent_id;
                if (!read_string(task_id)) return false;
                if (!read_string(agent_id)) return false;
                winners[task_id] = agent_id;
            }
            return true;
        }

        bool read_agent_timestamps(AgentTimestamps &timestamps) {
            uint32_t count;
            if (!read_uint32(count)) return false;
            timestamps.clear();
            for (uint32_t i = 0; i < count; ++i) {
                std::string agent_id;
                double ts;
                if (!read_string(agent_id)) return false;
                if (!read_double(ts)) return false;
                timestamps[agent_id] = ts;
            }
            return true;
        }
    };

    std::vector<uint8_t> CBBAMessage::serialize() const {
        BinaryWriter writer;

        // Message metadata
        writer.write_string(sender_id);
        writer.write_double(timestamp);

        // Bundle (just task IDs)
        writer.write_task_ids(bundle.get_tasks());

        // Path (just task IDs in order)
        writer.write_task_ids(path.get_tasks());

        // Winning bids
        writer.write_task_bids(winning_bids);

        // Winners
        writer.write_task_winners(winners);

        // Agent timestamps
        writer.write_agent_timestamps(timestamps);

        return writer.get_buffer();
    }

    bool CBBAMessage::deserialize(const std::vector<uint8_t> &data) {
        BinaryReader reader(data);

        // Message metadata
        if (!reader.read_string(sender_id)) return false;
        if (!reader.read_double(timestamp)) return false;

        // Bundle
        std::vector<TaskID> bundle_tasks;
        if (!reader.read_task_ids(bundle_tasks)) return false;

        // Clear existing bundle and add tasks
        // Bundle now has unlimited capacity by default
        bundle.clear();
        for (const auto &task_id : bundle_tasks) {
            bundle.add(task_id);
        }

        // Path
        std::vector<TaskID> path_tasks;
        if (!reader.read_task_ids(path_tasks)) return false;
        path.clear();
        for (size_t i = 0; i < path_tasks.size(); ++i) {
            path.insert(path_tasks[i], i);
        }

        // Winning bids
        if (!reader.read_task_bids(winning_bids)) return false;

        // Winners
        if (!reader.read_task_winners(winners)) return false;

        // Agent timestamps
        if (!reader.read_agent_timestamps(timestamps)) return false;

        return true;
    }

} // namespace consens::cbba
