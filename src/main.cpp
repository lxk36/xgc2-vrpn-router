#include <sys/time.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <vrpn_Connection.h>
#include <vrpn_Tracker.h>

namespace {

constexpr const char* kVersion = "0.1.0";
std::atomic_bool g_shutdown_requested{false};

void handleSignal(int) { g_shutdown_requested.store(true, std::memory_order_relaxed); }

void installSignalHandlers() {
    struct sigaction action {};
    action.sa_handler = handleSignal;
    sigemptyset(&action.sa_mask);
    action.sa_flags = 0;
    sigaction(SIGINT, &action, nullptr);
    sigaction(SIGTERM, &action, nullptr);
}

std::string trim(std::string value) {
    auto is_space = [](unsigned char ch) { return std::isspace(ch) != 0; };
    value.erase(value.begin(), std::find_if_not(value.begin(), value.end(), is_space));
    value.erase(std::find_if_not(value.rbegin(), value.rend(), is_space).base(), value.end());
    return value;
}

std::string stripInlineComment(const std::string& line) {
    bool in_quote = false;
    char quote = '\0';
    for (size_t i = 0; i < line.size(); ++i) {
        const char ch = line[i];
        if ((ch == '"' || ch == '\'') && (i == 0 || line[i - 1] != '\\')) {
            if (!in_quote) {
                in_quote = true;
                quote = ch;
            } else if (quote == ch) {
                in_quote = false;
            }
        }
        if (!in_quote && (ch == '#' || ch == ';')) {
            return line.substr(0, i);
        }
    }
    return line;
}

std::string unquote(std::string value) {
    value = trim(std::move(value));
    if (value.size() >= 2 &&
        ((value.front() == '"' && value.back() == '"') || (value.front() == '\'' && value.back() == '\''))) {
        return value.substr(1, value.size() - 2);
    }
    return value;
}

int parseInt(const std::string& value, const std::string& field) {
    char* end = nullptr;
    const long out = std::strtol(value.c_str(), &end, 10);
    if (end == value.c_str() || *end != '\0') {
        throw std::runtime_error(field + " must be an integer");
    }
    return static_cast<int>(out);
}

double parseDouble(const std::string& value, const std::string& field) {
    char* end = nullptr;
    const double out = std::strtod(value.c_str(), &end);
    if (end == value.c_str() || *end != '\0') {
        throw std::runtime_error(field + " must be a number");
    }
    return out;
}

struct TrackerConfig {
    std::string upstream;
    std::string downstream;
    int sensors{1};
};

struct RouterConfig {
    std::string config_path;
    std::string upstream_host{"127.0.0.1"};
    int upstream_port{3883};
    std::string bind_address;
    int listen_port{3883};
    double mainloop_rate_hz{240.0};
    double upstream_update_rate_hz{0.0};
    std::vector<TrackerConfig> trackers;
};

std::string upstreamHostSpec(const RouterConfig& config) {
    if (config.upstream_host.find(':') != std::string::npos) {
        return config.upstream_host;
    }
    return config.upstream_host + ":" + std::to_string(config.upstream_port);
}

std::string listenSpec(const RouterConfig& config) {
    if (config.bind_address.empty()) {
        return ":" + std::to_string(config.listen_port);
    }
    return config.bind_address + ":" + std::to_string(config.listen_port);
}

void validateConfig(const RouterConfig& config) {
    if (config.upstream_host.empty()) {
        throw std::runtime_error("UpstreamHost is required");
    }
    if (config.upstream_port <= 0 || config.upstream_port > 65535) {
        throw std::runtime_error("UpstreamPort must be in range 1..65535");
    }
    if (config.listen_port <= 0 || config.listen_port > 65535) {
        throw std::runtime_error("ListenPort must be in range 1..65535");
    }
    if (config.mainloop_rate_hz <= 0.0) {
        throw std::runtime_error("MainloopRate must be positive");
    }
    if (config.upstream_update_rate_hz < 0.0) {
        throw std::runtime_error("UpstreamUpdateRate must be non-negative");
    }
    if (config.trackers.empty()) {
        throw std::runtime_error("at least one tracker must be configured");
    }

    std::map<std::string, bool> downstream_names;
    for (const auto& tracker : config.trackers) {
        if (tracker.upstream.empty()) {
            throw std::runtime_error("tracker upstream name is required");
        }
        if (tracker.downstream.empty()) {
            throw std::runtime_error("tracker downstream name is required");
        }
        if (tracker.sensors <= 0) {
            throw std::runtime_error("tracker Sensors must be positive");
        }
        if (!downstream_names.emplace(tracker.downstream, true).second) {
            throw std::runtime_error("duplicate downstream tracker name: " + tracker.downstream);
        }
    }
}

TrackerConfig parseTrackerArgument(const std::string& value) {
    TrackerConfig tracker;
    const size_t equals = value.find('=');
    if (equals == std::string::npos) {
        tracker.upstream = trim(value);
        tracker.downstream = tracker.upstream;
        return tracker;
    }

    tracker.upstream = trim(value.substr(0, equals));
    std::string rhs = trim(value.substr(equals + 1));
    const size_t comma = rhs.find(',');
    if (comma == std::string::npos) {
        tracker.downstream = rhs;
    } else {
        tracker.downstream = trim(rhs.substr(0, comma));
        tracker.sensors = parseInt(trim(rhs.substr(comma + 1)), "--tracker sensors");
    }
    return tracker;
}

void applyKeyValue(RouterConfig& config, TrackerConfig* current_tracker, const std::string& key,
                   const std::string& value, int line_number) {
    const std::string field = "line " + std::to_string(line_number) + " key " + key;
    if (current_tracker != nullptr) {
        if (key == "Upstream") {
            current_tracker->upstream = value;
        } else if (key == "Downstream") {
            current_tracker->downstream = value;
        } else if (key == "Sensors") {
            current_tracker->sensors = parseInt(value, field);
        } else {
            throw std::runtime_error(field + " is not valid in a Tracker section");
        }
        return;
    }

    if (key == "UpstreamHost") {
        config.upstream_host = value;
    } else if (key == "UpstreamPort") {
        config.upstream_port = parseInt(value, field);
    } else if (key == "BindAddress") {
        config.bind_address = value;
    } else if (key == "ListenPort") {
        config.listen_port = parseInt(value, field);
    } else if (key == "MainloopRate") {
        config.mainloop_rate_hz = parseDouble(value, field);
    } else if (key == "UpstreamUpdateRate") {
        config.upstream_update_rate_hz = parseDouble(value, field);
    } else {
        throw std::runtime_error(field + " is not valid in the General section");
    }
}

void loadConfigFile(RouterConfig& config, const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("failed to open config file: " + path);
    }

    TrackerConfig* current_tracker = nullptr;
    bool in_general = true;
    std::string line;
    int line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        line = trim(stripInlineComment(line));
        if (line.empty()) {
            continue;
        }

        if (line.front() == '[' && line.back() == ']') {
            const std::string section = trim(line.substr(1, line.size() - 2));
            if (section == "General") {
                current_tracker = nullptr;
                in_general = true;
                continue;
            }

            constexpr const char* tracker_prefix = "Tracker ";
            if (section.rfind(tracker_prefix, 0) == 0) {
                TrackerConfig tracker;
                tracker.downstream = trim(section.substr(std::string(tracker_prefix).size()));
                tracker.upstream = tracker.downstream;
                config.trackers.push_back(tracker);
                current_tracker = &config.trackers.back();
                in_general = false;
                continue;
            }
            throw std::runtime_error("line " + std::to_string(line_number) + " has unsupported section: " + section);
        }

        const size_t equals = line.find('=');
        if (equals == std::string::npos) {
            throw std::runtime_error("line " + std::to_string(line_number) + " must be key=value");
        }
        const std::string key = trim(line.substr(0, equals));
        const std::string value = unquote(line.substr(equals + 1));
        applyKeyValue(config, in_general ? nullptr : current_tracker, key, value, line_number);
    }
}

void printUsage(std::ostream& out) {
    out << "xgc2-vrpn-router " << kVersion << "\n"
        << "\n"
        << "Usage:\n"
        << "  xgc2-vrpn-router --config /etc/xgc2/vrpn-router/router.conf\n"
        << "  xgc2-vrpn-router --upstream-host 192.168.10.20 --bind-address 192.168.51.14 --tracker uav1=uav1\n"
        << "\n"
        << "Options:\n"
        << "  --config PATH                 Load INI configuration file\n"
        << "  --check-config                Validate configuration and exit\n"
        << "  --upstream-host HOST          Upstream VRPN host or host:port\n"
        << "  --upstream-port PORT          Upstream VRPN port when host has no port\n"
        << "  --bind-address ADDRESS        Local server bind address; empty means all interfaces\n"
        << "  --listen-port PORT            Local VRPN server port\n"
        << "  --mainloop-rate HZ            Router mainloop rate\n"
        << "  --upstream-update-rate HZ     Request upstream tracker update rate; 0 disables request\n"
        << "  --tracker UP[=DOWN[,SENSORS]] Add tracker mapping\n"
        << "  --version                     Print version and exit\n"
        << "  --help                        Print this help\n";
}

struct CliOptions {
    RouterConfig config;
    bool check_config{false};
    bool print_help{false};
    bool print_version{false};
};

struct ConfigOverrides {
    bool upstream_host{false};
    bool upstream_port{false};
    bool bind_address{false};
    bool listen_port{false};
    bool mainloop_rate_hz{false};
    bool upstream_update_rate_hz{false};
};

CliOptions parseArgs(int argc, char** argv) {
    CliOptions options;
    ConfigOverrides overrides;
    std::vector<std::string> tracker_args;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto require_value = [&](const std::string& option) -> std::string {
            if (i + 1 >= argc) {
                throw std::runtime_error(option + " requires a value");
            }
            return argv[++i];
        };

        if (arg == "--help" || arg == "-h") {
            options.print_help = true;
        } else if (arg == "--version") {
            options.print_version = true;
        } else if (arg == "--check-config") {
            options.check_config = true;
        } else if (arg == "--config" || arg == "-c") {
            options.config.config_path = require_value(arg);
        } else if (arg == "--upstream-host") {
            options.config.upstream_host = require_value(arg);
            overrides.upstream_host = true;
        } else if (arg == "--upstream-port") {
            options.config.upstream_port = parseInt(require_value(arg), arg);
            overrides.upstream_port = true;
        } else if (arg == "--bind-address") {
            options.config.bind_address = require_value(arg);
            overrides.bind_address = true;
        } else if (arg == "--listen-port") {
            options.config.listen_port = parseInt(require_value(arg), arg);
            overrides.listen_port = true;
        } else if (arg == "--mainloop-rate") {
            options.config.mainloop_rate_hz = parseDouble(require_value(arg), arg);
            overrides.mainloop_rate_hz = true;
        } else if (arg == "--upstream-update-rate") {
            options.config.upstream_update_rate_hz = parseDouble(require_value(arg), arg);
            overrides.upstream_update_rate_hz = true;
        } else if (arg == "--tracker") {
            tracker_args.push_back(require_value(arg));
        } else {
            throw std::runtime_error("unknown argument: " + arg);
        }
    }

    if (!options.config.config_path.empty()) {
        RouterConfig file_config;
        file_config.config_path = options.config.config_path;
        loadConfigFile(file_config, options.config.config_path);
        const RouterConfig cli_config = options.config;
        options.config = file_config;
        if (overrides.upstream_host) {
            options.config.upstream_host = cli_config.upstream_host;
        }
        if (overrides.upstream_port) {
            options.config.upstream_port = cli_config.upstream_port;
        }
        if (overrides.bind_address) {
            options.config.bind_address = cli_config.bind_address;
        }
        if (overrides.listen_port) {
            options.config.listen_port = cli_config.listen_port;
        }
        if (overrides.mainloop_rate_hz) {
            options.config.mainloop_rate_hz = cli_config.mainloop_rate_hz;
        }
        if (overrides.upstream_update_rate_hz) {
            options.config.upstream_update_rate_hz = cli_config.upstream_update_rate_hz;
        }
    }

    for (const std::string& tracker_arg : tracker_args) {
        options.config.trackers.push_back(parseTrackerArgument(tracker_arg));
    }
    return options;
}

struct TrackerRoute {
    TrackerConfig config;
    std::unique_ptr<vrpn_Tracker_Remote> remote;
    std::unique_ptr<vrpn_Tracker_Server> server;
    std::uint64_t pose_count{0};
    std::uint64_t velocity_count{0};
    std::uint64_t acceleration_count{0};
};

class VrpnRouter {
  public:
    explicit VrpnRouter(RouterConfig config) : config_(std::move(config)) {
        validateConfig(config_);
        server_connection_ = vrpn_create_server_connection(listenSpec(config_).c_str());
        if (server_connection_ == nullptr) {
            throw std::runtime_error("failed to create downstream VRPN server connection at " + listenSpec(config_));
        }

        upstream_connection_ = vrpn_get_connection_by_name(upstreamHostSpec(config_).c_str());
        if (upstream_connection_ == nullptr) {
            throw std::runtime_error("failed to create upstream VRPN client connection to " +
                                     upstreamHostSpec(config_));
        }

        for (const TrackerConfig& tracker_config : config_.trackers) {
            auto route = std::make_unique<TrackerRoute>();
            route->config = tracker_config;
            route->server = std::make_unique<vrpn_Tracker_Server>(tracker_config.downstream.c_str(), server_connection_,
                                                                  tracker_config.sensors);
            route->remote =
                std::make_unique<vrpn_Tracker_Remote>(tracker_config.upstream.c_str(), upstream_connection_);
            route->remote->register_change_handler(route.get(), &VrpnRouter::handlePose);
            route->remote->register_change_handler(route.get(), &VrpnRouter::handleVelocity);
            route->remote->register_change_handler(route.get(), &VrpnRouter::handleAcceleration);
            if (config_.upstream_update_rate_hz > 0.0) {
                route->remote->set_update_rate(config_.upstream_update_rate_hz);
            }
            routes_.push_back(std::move(route));
        }
    }

    ~VrpnRouter() {
        routes_.clear();
        if (upstream_connection_ != nullptr) {
            upstream_connection_->removeReference();
            upstream_connection_ = nullptr;
        }
        if (server_connection_ != nullptr) {
            server_connection_->removeReference();
            server_connection_ = nullptr;
        }
    }

    void run() {
        std::cout << "xgc2-vrpn-router serving " << routes_.size() << " tracker(s), upstream "
                  << upstreamHostSpec(config_) << ", downstream " << listenSpec(config_) << std::endl;
        const auto sleep_duration = std::chrono::duration<double>(1.0 / std::max(config_.mainloop_rate_hz, 1.0));

        while (!g_shutdown_requested.load(std::memory_order_relaxed)) {
            for (auto& route : routes_) {
                route->remote->mainloop();
                route->server->mainloop();
            }
            server_connection_->mainloop();
            std::this_thread::sleep_for(sleep_duration);
        }
    }

  private:
    static bool sensorInRange(const TrackerRoute& route, vrpn_int32 sensor) {
        return sensor >= 0 && sensor < route.config.sensors;
    }

    static void VRPN_CALLBACK handlePose(void* userdata, const vrpn_TRACKERCB info) {
        auto* route = static_cast<TrackerRoute*>(userdata);
        if (!sensorInRange(*route, info.sensor)) {
            return;
        }
        if (route->server->report_pose(info.sensor, info.msg_time, info.pos, info.quat) == 0) {
            ++route->pose_count;
        }
    }

    static void VRPN_CALLBACK handleVelocity(void* userdata, const vrpn_TRACKERVELCB info) {
        auto* route = static_cast<TrackerRoute*>(userdata);
        if (!sensorInRange(*route, info.sensor)) {
            return;
        }
        if (route->server->report_pose_velocity(info.sensor, info.msg_time, info.vel, info.vel_quat,
                                                info.vel_quat_dt) == 0) {
            ++route->velocity_count;
        }
    }

    static void VRPN_CALLBACK handleAcceleration(void* userdata, const vrpn_TRACKERACCCB info) {
        auto* route = static_cast<TrackerRoute*>(userdata);
        if (!sensorInRange(*route, info.sensor)) {
            return;
        }
        if (route->server->report_pose_acceleration(info.sensor, info.msg_time, info.acc, info.acc_quat,
                                                    info.acc_quat_dt) == 0) {
            ++route->acceleration_count;
        }
    }

    RouterConfig config_;
    vrpn_Connection* server_connection_{nullptr};
    vrpn_Connection* upstream_connection_{nullptr};
    std::vector<std::unique_ptr<TrackerRoute>> routes_;
};

} // namespace

int main(int argc, char** argv) {
    try {
        CliOptions options = parseArgs(argc, argv);
        if (options.print_help) {
            printUsage(std::cout);
            return 0;
        }
        if (options.print_version) {
            std::cout << "xgc2-vrpn-router " << kVersion << std::endl;
            return 0;
        }
        validateConfig(options.config);
        if (options.check_config) {
            std::cout << "configuration ok: " << options.config.trackers.size() << " tracker(s)" << std::endl;
            return 0;
        }

        installSignalHandlers();
        VrpnRouter router(std::move(options.config));
        router.run();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "xgc2-vrpn-router: " << e.what() << std::endl;
        return 1;
    }
}
