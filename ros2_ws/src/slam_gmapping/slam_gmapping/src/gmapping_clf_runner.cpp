// Offline CLF (CARMEN log format) runner for GMapping's GridSlamProcessor.
//
// Counterpart of fastslam's clf_runner: feeds every laser entry of a .clf
// file to the filter sequentially — no ROS, no replay timers, no message
// queues — so runs are unaffected by processing speed and never drop scans.
// With --seed fixed, runs are bit-for-bit reproducible (GMapping's RNG is
// seeded through sampleGaussian and the library is single-threaded).
//
// Filter setup mirrors slam_gmapping.cpp (the ROS node) and its parameter
// values from clf_publisher/launch/dataset_gmapping.launch.py; CLF parsing
// mirrors clf_publisher/clf_node.py (raw odometry pose triplet, FLASER
// scan-parameter assumptions, monotonic timestamp clamping).

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <gmapping/gridfastslam/gridslamprocessor.h>
#include <gmapping/sensor/sensor_range/rangesensor.h>
#include <gmapping/sensor/sensor_range/rangereading.h>
#include <gmapping/utils/stat.h>

namespace {

struct ClfEntry {
    std::vector<double> ranges;
    double angle_min = 0.0, angle_increment = 0.0;
    double range_min = 0.0, range_max = 0.0;
    GMapping::OrientedPoint odom_pose;
    double timestamp = 0.0;
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
                e.ranges.reserve(n);
                for (int i = 0; i < n; i++)
                    e.ranges.push_back(std::stod(tokens.at(2 + i)));

                size_t off = 2 + n;
                e.odom_pose.x = std::stod(tokens.at(off + 3));
                e.odom_pose.y = std::stod(tokens.at(off + 4));
                e.odom_pose.theta = std::stod(tokens.at(off + 5));
                e.timestamp = std::stod(tokens.at(off + 6));

                // FLASER doesn't encode scan params — assume 180° FOV,
                // SICK LMS200-style limits (same as clf_node.py)
                e.angle_min = -M_PI / 2.0;
                e.angle_increment = M_PI / n;
                e.range_min = 0.02;
                e.range_max = 20.0;
            } else if (tokens[0] == "ROBOTLASER1") {
                double start_angle = std::stod(tokens.at(2));
                double angular_res = std::stod(tokens.at(4));
                double max_range = std::stod(tokens.at(5));
                int n = std::stoi(tokens.at(8));

                e.ranges.reserve(n);
                for (int i = 0; i < n; i++)
                    e.ranges.push_back(std::stod(tokens.at(9 + i)));

                size_t off = 9 + n;
                int num_remissions = std::stoi(tokens.at(off));
                off += 1 + num_remissions;

                // skip laser pose (off+0..2), take robot pose (off+3..5)
                e.odom_pose.x = std::stod(tokens.at(off + 3));
                e.odom_pose.y = std::stod(tokens.at(off + 4));
                e.odom_pose.theta = std::stod(tokens.at(off + 5));
                e.timestamp = std::stod(tokens.at(tokens.size() - 3));

                e.angle_min = start_angle;
                e.angle_increment = angular_res;
                e.range_min = 0.02;
                e.range_max = max_range;
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
    // monotonic — clamp offenders to prev + 1 ms (same as clf_node.py)
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

// Minimal parser for the flat `slam_gmapping: ros__parameters:` block of
// configs/gmapping/default.yaml — same config file the ROS node loads.
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
            in_section = (trimmed == "slam_gmapping:");
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

struct StampedPose {
    double t;
    GMapping::OrientedPoint pose;
};

void writeTumFile(const std::string& path, const std::vector<StampedPose>& poses) {
    std::ofstream f(path);
    if (!f) {
        std::cerr << "Failed to write " << path << "\n";
        return;
    }
    f << std::fixed << std::setprecision(6);
    for (const StampedPose& sp : poses) {
        f << sp.t << " " << sp.pose.x << " " << sp.pose.y << " 0 0 0 "
          << std::sin(sp.pose.theta / 2.0) << " " << std::cos(sp.pose.theta / 2.0) << "\n";
    }
}

// Rebuild the best particle's map from its trajectory tree (the same walk
// slam_gmapping's updateMap does) and write map.pgm/map.yaml in nav2
// map_saver conventions (occupied -> 0, free -> 254, unknown -> 205).
void writeMap(GMapping::GridSlamProcessor& gsp,
              const std::vector<double>& laser_angles,
              const GMapping::RangeSensor* laser,
              double max_range, double max_urange,
              double xmin, double ymin, double xmax, double ymax,
              double delta, double occ_thresh, const std::string& dir) {
    GMapping::ScanMatcher matcher;
    matcher.setLaserParameters(static_cast<unsigned int>(laser_angles.size()),
                               const_cast<double*>(laser_angles.data()),
                               laser->getPose());
    matcher.setlaserMaxRange(max_range);
    matcher.setusableRange(max_urange);
    matcher.setgenerateMap(true);

    const auto& best = gsp.getParticles()[gsp.getBestParticleIndex()];
    GMapping::Point center((xmin + xmax) / 2.0, (ymin + ymax) / 2.0);
    GMapping::ScanMatcherMap smap(center, xmin, ymin, xmax, ymax, delta);

    for (GMapping::GridSlamProcessor::TNode* n = best.node; n; n = n->parent) {
        if (!n->reading) continue;
        matcher.invalidateActiveArea();
        matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
        matcher.registerScan(smap, n->pose, &((*n->reading)[0]));
    }

    int width = smap.getMapSizeX();
    int height = smap.getMapSizeY();
    GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));

    std::ofstream pgm(dir + "/map.pgm", std::ios::binary);
    if (!pgm) {
        std::cerr << "Failed to write " << dir << "/map.pgm\n";
        return;
    }
    pgm << "P5\n" << width << " " << height << "\n255\n";
    // PGM rows go top to bottom; map rows go bottom (origin) to top
    for (int y = height - 1; y >= 0; y--) {
        for (int x = 0; x < width; x++) {
            double occ = smap.cell(GMapping::IntPoint(x, y));
            unsigned char pixel = 205;
            if (occ >= 0)
                pixel = (occ > occ_thresh) ? 0 : 254;
            pgm.write(reinterpret_cast<const char*>(&pixel), 1);
        }
    }
    pgm.close();

    std::ofstream yaml(dir + "/map.yaml");
    yaml << "image: map.pgm\n"
         << "resolution: " << delta << "\n"
         << "origin: [" << wmin.x << ", " << wmin.y << ", 0.0]\n"
         << "negate: 0\n"
         << "occupied_thresh: 0.65\n"
         << "free_thresh: 0.196\n";
}

void printUsage(const char* prog) {
    std::cerr << "Usage: " << prog << " <dataset.clf> [--params <yaml>] [--out <dir>] [--seed <n>]\n"
              << "  [--particles <n>] [--delta <m>] [--linear-update <m>]\n"
              << "  [--angular-update <rad>] [--resample-threshold <f>]\n"
              << "  [--maxurange <m>] [--minimum-score <f>]\n";
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }
    std::string clf_file = argv[1];
    std::string out_dir = "results";
    long seed = -1;

    // Filter-level parameters: defaults from dataset_gmapping.launch.py.
    // Scan matcher / likelihood internals: GMapping defaults, as in the node.
    int particles = 60;
    double delta = 0.1;
    double linear_update = 0.25;
    double angular_update = 0.1;
    double resample_threshold = 0.5;
    double srr = 0.1, srt = 0.2, str = 0.1, stt = 0.2;
    double max_range = 0.0, max_urange = 0.0;   // <= 0: derived from scan range_max
    double temporal_update = -1.0;
    double xmin = -20.0, ymin = -20.0, xmax = 20.0, ymax = 20.0;
    double sigma = 0.05, lsigma = 0.075, ogain = 3.0;
    int kernel_size = 1, iterations = 5, lskip = 0;
    double lstep = 0.05, astep = 0.05;
    double llsamplerange = 0.01, llsamplestep = 0.01;
    double lasamplerange = 0.005, lasamplestep = 0.005;
    double minimum_score = 0.0, occ_thresh = 0.25;

    // Apply --params yaml first so the remaining CLI flags override it
    for (int i = 2; i + 1 < argc; i++) {
        if (std::string(argv[i]) == "--params") {
            std::map<std::string, std::string> p = parseParamsYaml(argv[i + 1]);
            auto getd = [&](const char* k, double d) {
                auto it = p.find(k); return it != p.end() ? std::stod(it->second) : d;
            };
            auto geti = [&](const char* k, int d) {
                auto it = p.find(k); return it != p.end() ? std::stoi(it->second) : d;
            };
            particles = geti("particles", particles);
            delta = getd("delta", delta);
            linear_update = getd("linearUpdate", linear_update);
            angular_update = getd("angularUpdate", angular_update);
            resample_threshold = getd("resampleThreshold", resample_threshold);
            srr = getd("srr", srr);
            srt = getd("srt", srt);
            str = getd("str", str);
            stt = getd("stt", stt);
            max_range = getd("maxRange", max_range);
            max_urange = getd("maxUrange", max_urange);
            temporal_update = getd("temporalUpdate", temporal_update);
            xmin = getd("xmin", xmin);
            ymin = getd("ymin", ymin);
            xmax = getd("xmax", xmax);
            ymax = getd("ymax", ymax);
            sigma = getd("sigma", sigma);
            lsigma = getd("lsigma", lsigma);
            ogain = getd("ogain", ogain);
            kernel_size = geti("kernelSize", kernel_size);
            iterations = geti("iterations", iterations);
            lskip = geti("lskip", lskip);
            lstep = getd("lstep", lstep);
            astep = getd("astep", astep);
            llsamplerange = getd("llsamplerange", llsamplerange);
            llsamplestep = getd("llsamplestep", llsamplestep);
            lasamplerange = getd("lasamplerange", lasamplerange);
            lasamplestep = getd("lasamplestep", lasamplestep);
            minimum_score = getd("minimumScore", minimum_score);
            occ_thresh = getd("occ_thresh", occ_thresh);
        }
    }

    for (int i = 2; i < argc; i++) {
        std::string arg = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) { printUsage(argv[0]); exit(1); }
            return argv[++i];
        };
        if (arg == "--params") { next(); }  // already applied above
        else if (arg == "--out") out_dir = next();
        else if (arg == "--seed") seed = std::stol(next());
        else if (arg == "--particles") particles = std::stoi(next());
        else if (arg == "--delta") delta = std::stod(next());
        else if (arg == "--linear-update") linear_update = std::stod(next());
        else if (arg == "--angular-update") angular_update = std::stod(next());
        else if (arg == "--resample-threshold") resample_threshold = std::stod(next());
        else if (arg == "--maxurange") max_urange = std::stod(next());
        else if (arg == "--minimum-score") minimum_score = std::stod(next());
        else { printUsage(argv[0]); return 1; }
    }

    std::vector<ClfEntry> entries = parseClf(clf_file);
    if (entries.empty()) {
        std::cerr << "No laser entries found in " << clf_file << "\n";
        return 1;
    }
    std::cout << "Loaded " << entries.size() << " laser readings from " << clf_file << "\n";

    const ClfEntry& first = entries[0];
    unsigned int beam_count = static_cast<unsigned int>(first.ranges.size());
    if (max_range <= 0.0) max_range = first.range_max - 0.01;
    if (max_urange <= 0.0) max_urange = max_range;

    // The RangeSensor assumes beams symmetric about the sensor axis; any
    // asymmetry of the actual FOV is folded into the reading pose below
    // (the ROS node does the same via its "centered laser pose" TF).
    double fov = beam_count * first.angle_increment;
    double angle_center = first.angle_min + fov / 2.0;

    // The laser must be called "FLASER" (GMapping convention)
    auto* laser = new GMapping::RangeSensor(
        "FLASER", beam_count, first.angle_increment,
        GMapping::OrientedPoint(0, 0, 0), 0.0, max_range);

    std::vector<double> laser_angles(beam_count);
    double theta = -fov / 2.0;
    for (unsigned int i = 0; i < beam_count; i++) {
        laser_angles[i] = theta;
        theta += first.angle_increment;
    }

    GMapping::GridSlamProcessor gsp;
    GMapping::SensorMap smap;
    smap.insert(std::make_pair(laser->getName(), laser));
    gsp.setSensorMap(smap);

    gsp.setMatchingParameters(max_urange, max_range, sigma, kernel_size,
                              lstep, astep, iterations, lsigma, ogain,
                              static_cast<unsigned int>(lskip));
    gsp.setMotionModelParameters(srr, srt, str, stt);
    gsp.setUpdateDistances(linear_update, angular_update, resample_threshold);
    gsp.setUpdatePeriod(temporal_update);
    gsp.setgenerateMap(false);
    gsp.GridSlamProcessor::init(static_cast<unsigned int>(particles),
                                xmin, ymin, xmax, ymax, delta, first.odom_pose);
    gsp.setllsamplerange(llsamplerange);
    gsp.setllsamplestep(llsamplestep);
    gsp.setlasamplerange(lasamplerange);
    gsp.setlasamplestep(lasamplestep);
    gsp.setminimumScore(minimum_score);

    unsigned int used_seed = seed >= 0 ? static_cast<unsigned int>(seed)
                                       : static_cast<unsigned int>(time(nullptr));
    // Call the sampling function once to set the seed (as the ROS node does)
    GMapping::sampleGaussian(1, used_seed);

    std::cout << "particles: " << particles << " | delta: " << delta
              << " | seed: " << used_seed << "\n";

    auto t_start = std::chrono::steady_clock::now();
    std::vector<StampedPose> traj, odom_traj;
    std::vector<double> ranges(beam_count);
    int updates = 0, skipped_beams = 0;

    for (size_t k = 0; k < entries.size(); k++) {
        const ClfEntry& e = entries[k];
        if (e.ranges.size() != beam_count) {  // sensor must stay constant
            skipped_beams++;
            continue;
        }
        // Short readings must be filtered out, the mapper won't (node behavior)
        for (unsigned int i = 0; i < beam_count; i++)
            ranges[i] = e.ranges[i] < e.range_min ? e.range_max : e.ranges[i];

        GMapping::RangeReading reading(beam_count, ranges.data(), laser, e.timestamp);
        GMapping::OrientedPoint laser_pose = e.odom_pose;
        laser_pose.theta += angle_center;  // usually 0 (symmetric FOV)
        reading.setPose(laser_pose);

        auto t0 = std::chrono::steady_clock::now();
        bool updated = gsp.processScan(reading);
        double ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0).count();

        if (!updated) continue;
        updates++;
        GMapping::OrientedPoint best =
            gsp.getParticles()[gsp.getBestParticleIndex()].pose;
        best.theta -= angle_center;
        traj.push_back({e.timestamp, best});
        odom_traj.push_back({e.timestamp, e.odom_pose});

        std::printf("[%d] %.1fms | neff=%.1f/%d | best=(%.2f,%.2f,%.2f) (entry %zu/%zu)\n",
                    updates, ms, gsp.getneff(), particles,
                    best.x, best.y, best.theta, k + 1, entries.size());
    }
    double elapsed_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t_start).count();

    if (skipped_beams)
        std::cerr << "Skipped " << skipped_beams << " entries with a different beam count\n";

    std::error_code ec;
    std::filesystem::create_directories(out_dir, ec);
    writeTumFile(out_dir + "/gmapping.tum", traj);
    writeTumFile(out_dir + "/odom.tum", odom_traj);
    writeMap(gsp, laser_angles, laser, max_range, max_urange,
             xmin, ymin, xmax, ymax, delta, occ_thresh, out_dir);

    std::printf("\nDone: %zu entries, %d filter updates in %.1fs\n",
                entries.size(), updates, elapsed_s);
    std::printf("Wrote %s/gmapping.tum, odom.tum, map.pgm, map.yaml\n", out_dir.c_str());
    return 0;
}
