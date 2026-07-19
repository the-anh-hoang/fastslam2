#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "fastslam/fastslam2.hpp"
#include "fastslam/laser_scan.hpp"
#include "fastslam/pose.hpp"

namespace {

struct ClfEntry {
    fastslam::LaserScan scan;
    fastslam::Pose odom_pose;
    double timestamp;
};

std::vector<ClfEntry> parseClf(const std::string& path) {
    std::vector<ClfEntry> entries;
    std::ifstream f(path);
    if (!f) {
        std::cerr << "Failed to open " << path << "\n";
        return entries;
    }

    std::string line;
    int malformed = 0;
    while (std::getline(f, line)) {
        std::istringstream ss(line);
        std::vector<std::string> tokens;
        std::string tok;
        while (ss >> tok) tokens.push_back(tok);
        if (tokens.empty()) continue;

        try {
            ClfEntry e;
            if (tokens[0] == "FLASER") {
                // FLASER n r1..rn <corrected laser pose x y θ — skip>
                // <raw odom x y θ> <timestamp> ...
                int n = std::stoi(tokens.at(1));
                e.scan.ranges.reserve(n);
                for (int i = 0; i < n; i++)
                    e.scan.ranges.push_back(std::stof(tokens.at(2 + i)));

                size_t off = 2 + n;
                e.odom_pose.x = std::stod(tokens.at(off + 3));
                e.odom_pose.y = std::stod(tokens.at(off + 4));
                e.odom_pose.theta = std::stod(tokens.at(off + 5));
                e.timestamp = std::stod(tokens.at(off + 6));

                e.scan.angle_min = -M_PI / 2.0;
                e.scan.angle_increment = M_PI / n;
                e.scan.range_min = 0.02;
                e.scan.range_max = 20.0;
            } else if (tokens[0] == "ROBOTLASER1") {
                double start_angle = std::stod(tokens.at(2));
                double angular_res = std::stod(tokens.at(4));
                double max_range = std::stod(tokens.at(5));
                int n = std::stoi(tokens.at(8));

                e.scan.ranges.reserve(n);
                for (int i = 0; i < n; i++)
                    e.scan.ranges.push_back(std::stof(tokens.at(9 + i)));

                size_t off = 9 + n;
                int num_remissions = std::stoi(tokens.at(off));
                off += 1 + num_remissions;

                // skip laser pose (off+0..2), take robot pose (off+3..5)
                e.odom_pose.x = std::stod(tokens.at(off + 3));
                e.odom_pose.y = std::stod(tokens.at(off + 4));
                e.odom_pose.theta = std::stod(tokens.at(off + 5));
                e.timestamp = std::stod(tokens.at(tokens.size() - 3));

                e.scan.angle_min = start_angle;
                e.scan.angle_increment = angular_res;
                e.scan.range_min = 0.02;
                e.scan.range_max = max_range;
            } else {
                continue;
            }
            entries.push_back(std::move(e));
        } catch (const std::exception&) {
            malformed++;
        }
    }
    if (malformed)
        std::cerr << "Skipped " << malformed << " malformed lines\n";

    // CARMEN logs interleave sensor clocks, so timestamps are not always
    // monotonic — clamp offenders to prev + 1 ms
    int clamped = 0;
    for (size_t i = 1; i < entries.size(); i++) {
        if (entries[i].timestamp <= entries[i - 1].timestamp) {
            entries[i].timestamp = entries[i - 1].timestamp + 1e-3;
            clamped++;
        }
    }
    if (clamped)
        std::cout << "Clamped " << clamped << " non-monotonic timestamps\n";
    return entries;
}

// Minimal parser for the flat `fastslam2_node: ros__parameters:` block of
// fastslam_params.yaml — enough to share one config file with the ROS node.
std::map<std::string, std::string> parseParamsYaml(const std::string& path) {
    std::map<std::string, std::string> params;
    std::ifstream f(path);
    if (!f) {
        std::cerr << "Failed to open params file " << path << " — using defaults\n";
        return params;
    }
    std::string line;
    bool in_section = false;
    while (std::getline(f, line)) {
        size_t comment = line.find('#');
        if (comment != std::string::npos) line = line.substr(0, comment);

        size_t start = line.find_first_not_of(" \t");
        if (start == std::string::npos) continue;
        size_t end = line.find_last_not_of(" \t\r");
        bool top_level = (start == 0);
        std::string trimmed = line.substr(start, end - start + 1);

        if (top_level) {
            in_section = (trimmed == "fastslam2_node:");
            continue;
        }
        if (!in_section || trimmed == "ros__parameters:") continue;

        size_t colon = trimmed.find(':');
        if (colon == std::string::npos) continue;
        std::string key = trimmed.substr(0, colon);
        std::string value = trimmed.substr(colon + 1);
        size_t vstart = value.find_first_not_of(" \t\"'");
        if (vstart == std::string::npos) continue;
        size_t vend = value.find_last_not_of(" \t\"'");
        params[key] = value.substr(vstart, vend - vstart + 1);
    }
    return params;
}

fastslam::FastSlam2Config configFromParams(const std::map<std::string, std::string>& p) {
    fastslam::FastSlam2Config c;
    auto getd = [&](const char* k, double d) {
        auto it = p.find(k); return it != p.end() ? std::stod(it->second) : d;
    };
    auto geti = [&](const char* k, int d) {
        auto it = p.find(k); return it != p.end() ? std::stoi(it->second) : d;
    };
    c.num_particles = geti("num_particles", c.num_particles);
    c.map_chunk_size = static_cast<float>(getd("map_chunk_size", c.map_chunk_size));
    c.map_res = static_cast<float>(getd("map_res", c.map_res));
    if (auto it = p.find("mode"); it != p.end()) c.mode = it->second;
    if (auto it = p.find("matcher"); it != p.end()) c.matcher = it->second;
    c.neff_gain = getd("neff_gain", c.neff_gain);
    c.resample_gain = getd("resample_gain", c.resample_gain);
    c.proposal_range_xy = getd("proposal_range_xy", c.proposal_range_xy);
    c.proposal_step_xy = getd("proposal_step_xy", c.proposal_step_xy);
    c.proposal_range_theta = getd("proposal_range_theta", c.proposal_range_theta);
    c.proposal_step_theta = getd("proposal_step_theta", c.proposal_step_theta);
    c.a1 = getd("a1", c.a1);
    c.a2 = getd("a2", c.a2);
    c.a3 = getd("a3", c.a3);
    c.a4 = getd("a4", c.a4);
    c.ray_skip = geti("ray_skip", c.ray_skip);
    c.scan_match_x_range = getd("scan_match_x_range", c.scan_match_x_range);
    c.scan_match_y_range = getd("scan_match_y_range", c.scan_match_y_range);
    c.scan_match_theta_range = getd("scan_match_theta_range", c.scan_match_theta_range);
    c.scan_match_step_xy = getd("scan_match_step_xy", c.scan_match_step_xy);
    c.scan_match_step_theta = getd("scan_match_step_theta", c.scan_match_step_theta);
    c.z_hit = getd("z_hit", c.z_hit);
    c.std_hit = getd("std_hit", c.std_hit);
    c.z_rand = getd("z_rand", c.z_rand);
    c.linear_update = getd("linear_update", c.linear_update);
    c.angular_update = getd("angular_update", c.angular_update);
    c.resample_threshold = getd("resample_threshold", c.resample_threshold);
    return c;
}

// nav2 map_saver conventions: occupied -> 0 (black), unknown -> 205 (gray)
void writeMap(const fastslam::OccupancyGridMap& map, const std::string& dir) {
    fastslam::GridData grid = map.toGridData();
    fastslam::MapParams params = map.getMapParams();

    std::ofstream pgm(dir + "/map.pgm", std::ios::binary);
    if (!pgm) {
        std::cerr << "Failed to write " << dir << "/map.pgm\n";
        return;
    }
    pgm << "P5\n" << grid.width << " " << grid.height << "\n255\n";
    // PGM rows go top to bottom; grid rows go bottom (origin) to top
    for (int y = grid.height - 1; y >= 0; y--) {
        for (int x = 0; x < grid.width; x++) {
            int8_t v = grid.data[y * grid.width + x];
            unsigned char pixel = (v == 100) ? 0 : (v == 0 ? 254 : 205);
            pgm.write(reinterpret_cast<const char*>(&pixel), 1);
        }
    }
    pgm.close();

    std::ofstream yaml(dir + "/map.yaml");
    yaml << "image: map.pgm\n"
         << "resolution: " << params.resolution << "\n"
         << "origin: [" << grid.origin_x << ", " << grid.origin_y << ", 0.0]\n"
         << "negate: 0\n"
         << "occupied_thresh: 0.65\n"
         << "free_thresh: 0.196\n";
}

void printUsage(const char* prog) {
    std::cerr << "Usage: " << prog << " <dataset.clf>"
              << " [--params <yaml>] [--out <dir>] [--seed <n>]"
              << " [--laser-dx <m>] [--laser-dy <m>] [--laser-dtheta <rad>]\n";
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }
    std::string clf_file = argv[1];
    std::string params_file = "configs/fastslam/default.yaml";
    std::string out_dir = "results";
    long seed = -1;
    double laser_dx = 0.0, laser_dy = 0.0, laser_dtheta = 0.0;

    for (int i = 2; i < argc; i++) {
        std::string arg = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) { printUsage(argv[0]); exit(1); }
            return argv[++i];
        };
        if (arg == "--params") params_file = next();
        else if (arg == "--out") out_dir = next();
        else if (arg == "--seed") seed = std::stol(next());
        else if (arg == "--laser-dx") laser_dx = std::stod(next());
        else if (arg == "--laser-dy") laser_dy = std::stod(next());
        else if (arg == "--laser-dtheta") laser_dtheta = std::stod(next());
        else { printUsage(argv[0]); return 1; }
    }

    std::vector<ClfEntry> entries = parseClf(clf_file);
    if (entries.empty()) {
        std::cerr << "No laser entries found in " << clf_file << "\n";
        return 1;
    }
    std::cout << "Loaded " << entries.size() << " laser readings from " << clf_file << "\n";

    fastslam::FastSlam2Config config = configFromParams(parseParamsYaml(params_file));
    config.laser_dx = laser_dx;
    config.laser_dy = laser_dy;
    config.laser_dtheta = laser_dtheta;
    config.seed = seed;
    std::cout << "num_particles: " << config.num_particles
              << " | mode: " << config.mode
              << " | seed: " << (seed >= 0 ? std::to_string(seed) : "random")
              << "\n";

    fastslam::FastSlam2 slam(config);

    auto t_start = std::chrono::steady_clock::now();
    int updates = 0;
    fastslam::UpdateResult last{};
    for (size_t k = 0; k < entries.size(); k++) {
        const ClfEntry& e = entries[k];
        fastslam::UpdateResult res = slam.processScan(e.scan, e.odom_pose, e.timestamp);
        if (res.status != fastslam::UpdateResult::Status::Updated) continue;
        updates++;
        last = res;

        std::printf("[%d] %.1fms | dist=%.1fm | N_eff=%.1f/%d (%.0f%%) | spread=%.4fm | %s (entry %zu/%zu)\n",
            res.scan_count, res.duration_ms, res.total_distance,
            res.n_eff, config.num_particles, res.n_eff_ratio * 100.0,
            res.spread,
            res.did_resample ? "RESAMPLED" : "", k + 1, entries.size());
        std::printf("  scan_match: ll=[%.1f, %.1f, %.1f] | proposal_cov: sx=%.5f sy=%.5f st=%.5f\n",
            res.min_ll, res.avg_ll, res.max_ll,
            std::sqrt(res.avg_cov_xx), std::sqrt(res.avg_cov_yy), std::sqrt(res.avg_cov_tt));
        std::printf("  weights: [%.4f, %.4f] | best=(%.2f,%.2f,%.2f) | resamples=%d/%d\n",
            res.w_min, res.w_max,
            res.best_pose.x, res.best_pose.y, res.best_pose.theta,
            res.resample_count, res.scan_count);
    }
    double elapsed_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t_start).count();

    std::error_code ec;
    std::filesystem::create_directories(out_dir, ec);
    slam.writeTrajectories(out_dir);
    writeMap(slam.bestParticle().map, out_dir);

    std::printf("\nDone: %zu entries, %d filter updates, %d resamples, %.1fm traveled in %.1fs\n",
        entries.size(), updates, last.resample_count, last.total_distance, elapsed_s);
    std::printf("Wrote %s/fastslam.tum, odom.tum, map.pgm, map.yaml\n", out_dir.c_str());
    return 0;
}
