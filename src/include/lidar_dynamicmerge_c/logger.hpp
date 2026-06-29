#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <mutex>
#include <chrono>
#include <iomanip>

namespace lidar_dynamic_merge {

enum class LogLevel {
    DEBUG,
    INFO,
    WARN,
    ERROR,
    FATAL
};

class Logger {
public:
    static Logger& getInstance() {
        static Logger instance;
        return instance;
    }

    void init(const std::string& log_file) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (file_stream_.is_open()) {
            file_stream_.close();
        }
        file_stream_.open(log_file, std::ios::out | std::ios::trunc);
        if (!file_stream_.is_open()) {
            std::cerr << "Failed to open log file: " << log_file << std::endl;
        }
    }

    void setLevel(LogLevel level) {
        current_level_ = level;
    }

    void log(LogLevel level, const char* file, int line, const std::string& message) {
        if (level < current_level_) return;

        std::string level_str;
        switch (level) {
            case LogLevel::DEBUG: level_str = "DEBUG"; break;
            case LogLevel::INFO:  level_str = "INFO"; break;
            case LogLevel::WARN:  level_str = "WARN"; break;
            case LogLevel::ERROR: level_str = "ERROR"; break;
            case LogLevel::FATAL: level_str = "FATAL"; break;
        }

        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now + std::chrono::hours(8));
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;

        std::ostringstream ss;
        ss << std::put_time(std::localtime(&time_t_now), "%Y-%m-%d %H:%M:%S")
           << '.' << std::setfill('0') << std::setw(3) << ms.count()
           << " [" << level_str << "] " << message;

        std::string log_msg = ss.str();

        std::lock_guard<std::mutex> lock(mutex_);
        if (level == LogLevel::ERROR || level == LogLevel::FATAL) {
            std::cerr << log_msg << std::endl;
        } else {
            std::cout << log_msg << std::endl;
        }
        
        if (file_stream_.is_open()) {
            file_stream_ << log_msg << std::endl;
            file_stream_.flush();
        }
        
        if (level == LogLevel::FATAL) {
            std::abort();
        }
    }

private:
    Logger() = default;
    ~Logger() {
        if (file_stream_.is_open()) {
            file_stream_.close();
        }
    }

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    std::ofstream file_stream_;
    std::mutex mutex_;
    LogLevel current_level_ = LogLevel::INFO;
};

} // namespace lidar_dynamic_merge

// Macros for ease of use
#define LOG_DEBUG(msg) do { std::ostringstream __ss; __ss << msg; lidar_dynamic_merge::Logger::getInstance().log(lidar_dynamic_merge::LogLevel::DEBUG, __FILE__, __LINE__, __ss.str()); } while(0)
#define LOG_INFO(msg)  do { std::ostringstream __ss; __ss << msg; lidar_dynamic_merge::Logger::getInstance().log(lidar_dynamic_merge::LogLevel::INFO,  __FILE__, __LINE__, __ss.str()); } while(0)
#define LOG_WARN(msg)  do { std::ostringstream __ss; __ss << msg; lidar_dynamic_merge::Logger::getInstance().log(lidar_dynamic_merge::LogLevel::WARN,  __FILE__, __LINE__, __ss.str()); } while(0)
#define LOG_ERROR(msg) do { std::ostringstream __ss; __ss << msg; lidar_dynamic_merge::Logger::getInstance().log(lidar_dynamic_merge::LogLevel::ERROR, __FILE__, __LINE__, __ss.str()); } while(0)
#define LOG_FATAL(msg) do { std::ostringstream __ss; __ss << msg; lidar_dynamic_merge::Logger::getInstance().log(lidar_dynamic_merge::LogLevel::FATAL, __FILE__, __LINE__, __ss.str()); } while(0)
