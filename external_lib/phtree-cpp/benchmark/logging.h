// Copyright (c) Improbable Worlds Ltd, All Rights Reserved
#ifndef PHTREE_BENCHMARK_LOGGING_H
#define PHTREE_BENCHMARK_LOGGING_H

#include <spdlog/spdlog.h>
#include <spdlog/sinks/sink.h>
#ifdef _WIN32
#include <spdlog/sinks/wincolor_sink.h>
#else
#include <spdlog/sinks/ansicolor_sink.h>
#endif

namespace improbable::phtree::phbenchmark::logging {

#ifdef _WIN32
using ConsoleSpdlogSink = spdlog::sinks::wincolor_stdout_sink_mt;
#else
using ConsoleSpdlogSink = spdlog::sinks::ansicolor_stdout_sink_mt;
#endif

constexpr auto kInternalLoggerName = "internal";

// Sets up spdlog for internal and external. If you need to do some logging before doing this
// call, use instead CaptureLogMessagesToBufferSink()/SetupLoggingAndFlushBuffer.
void SetupLogging(std::vector<spdlog::sink_ptr> sinks, spdlog::level::level_enum log_level) {
    auto& console_sink = sinks.emplace_back(std::make_shared<ConsoleSpdlogSink>());
    console_sink->set_level(log_level);

    // Find the minimum log level, in case one of the sinks passed to us has a lower log level.
    const auto& sink_with_lowest_log_level = *std::min_element(
        sinks.begin(),
        sinks.end(),
        [](const spdlog::sink_ptr& a, const spdlog::sink_ptr& b) -> bool {
            return a->level() < b->level();
        });
    spdlog::level::level_enum min_log_level =
        std::min(sink_with_lowest_log_level->level(), log_level);

    // Create the external logger, worker logger and the internal (default) logger from the same log
    // sinks. Each logsink can use `GetLoggerTypeFromMessage` to determine which logger a message
    // was logged to.
    spdlog::set_default_logger(
        std::make_shared<spdlog::logger>(kInternalLoggerName, sinks.begin(), sinks.end()));
    spdlog::set_level(min_log_level);
    spdlog::flush_on(min_log_level);
}

// Sets up default logging typically used for tests/benchmarks. Also used for default
// initialization if the logging hasn't been initialized before the first logging line.
void SetupDefaultLogging() {
    SetupLogging({}, spdlog::level::warn);
}

template <typename... Args>
inline void log(
    spdlog::source_loc source,
    spdlog::level::level_enum lvl,
    spdlog::string_view_t fmt,
    const Args&... args) {
    spdlog::log(source, lvl, fmt, args...);
}

template <typename... Args>
inline void log(spdlog::level::level_enum lvl, spdlog::string_view_t fmt, const Args&... args) {
    spdlog::log(spdlog::source_loc{}, lvl, fmt, args...);
}

template <typename... Args>
inline void trace(spdlog::string_view_t fmt, const Args&... args) {
    log(spdlog::level::level_enum::trace, fmt, args...);
}

template <typename... Args>
inline void debug(spdlog::string_view_t fmt, const Args&... args) {
    log(spdlog::level::level_enum::debug, fmt, args...);
}

template <typename... Args>
inline void info(spdlog::string_view_t fmt, const Args&... args) {
    log(spdlog::level::level_enum::info, fmt, args...);
}

template <typename... Args>
inline void warn(spdlog::string_view_t fmt, const Args&... args) {
    log(spdlog::level::level_enum::warn, fmt, args...);
}

template <typename... Args>
inline void error(spdlog::string_view_t fmt, const Args&... args) {
    log(spdlog::level::level_enum::err, fmt, args...);
}

template <typename... Args>
inline void critical(spdlog::string_view_t fmt, const Args&... args) {
    log(spdlog::level::level_enum::critical, fmt, args...);
}

template <typename T>
inline void log(spdlog::source_loc source, spdlog::level::level_enum lvl, const T& msg) {
    spdlog::log(source, lvl, msg);
}

template <typename T>
inline void log(spdlog::level::level_enum lvl, const T& msg) {
    spdlog::log(lvl, msg);
}

template <typename T>
inline void trace(const T& msg) {
    log(spdlog::level::level_enum::trace, msg);
}

template <typename T>
inline void debug(const T& msg) {
    log(spdlog::level::level_enum::debug, msg);
}

template <typename T>
inline void info(const T& msg) {
    log(spdlog::level::level_enum::info, msg);
}

template <typename T>
inline void warn(const T& msg) {
    log(spdlog::level::level_enum::warn, msg);
}

template <typename T>
inline void error(const T& msg) {
    log(spdlog::level::level_enum::err, msg);
}

template <typename T>
inline void critical(const T& msg) {
    log(spdlog::level::level_enum::critical, msg);
}

}  // namespace improbable::phtree::phbenchmark::logging

#endif  // PHTREE_BENCHMARK_LOGGING_H
