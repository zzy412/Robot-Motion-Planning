
#include "VehicleModel.h"
#include "GridMap.h"
#include "AStar.h"
#include "HybridAStar.h"
#include "Visualization.h"
#include "PurePursuit.h"
#include "AdaptivePurePursuit.h"
#include "MultiScenarioBenchmark.h"
#include "SteeringRateBenchmark.h"
#include "SpeedPlanningBenchmark.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

using namespace robot;
namespace fs = std::filesystem;

// ============================================
// 1. Save planned path
// ============================================

void savePath(
    const fs::path& name,
    const PlanResult& result)
{
    std::ofstream file(name);

    file << "x,y,theta\n";

    for (const auto& p : result.path)
    {
        file << p.x << ","
            << p.y << ","
            << p.theta << "\n";
    }
}

// ============================================
// 2. Ackermann vehicle demonstration
// ============================================

void vehicleDemo(
    const VehicleModel& vehicle,
    const fs::path& folder)
{
    std::ofstream file(
        folder / "vehicle_paths.csv"
    );

    file << "steering_deg,x,y,theta\n";

    for (double deg :
    { -30.0, -15.0, 0.0, 15.0, 30.0 })
    {
        State s{ 2.0, 2.0, 0.0 };

        for (int i = 0; i < 100; ++i)
        {
            s = vehicle.step(
                s,
                1.0,
                deg * PI / 180.0,
                0.05
            );

            file << deg << ","
                << s.x << ","
                << s.y << ","
                << s.theta << "\n";
        }

        std::cout
            << "steer " << deg
            << " deg -> ("
            << s.x << ", "
            << s.y << ")\n";
    }
}

// ============================================
// 3. Save tracking data
// ============================================

void saveTracking(
    const fs::path& name,
    const TrackingResult& result,
    const std::vector<double>* lookaheads = nullptr)
{
    std::ofstream file(name);

    file << "time,x,y,theta,steering,error,lookahead\n";

    for (size_t i = 0;
        i < result.samples.size();
        ++i)
    {
        const auto& q = result.samples[i];

        file << q.time << ","
            << q.state.x << ","
            << q.state.y << ","
            << q.state.theta << ","
            << q.steering << ","
            << q.crossTrackError << ",";

        if (lookaheads && i < lookaheads->size())
        {
            file << (*lookaheads)[i];
        }

        file << "\n";
    }
}

// ============================================
// 4. Point-to-segment distance
// ============================================

double pointToSegmentDistance(
    const State& p,
    const State& a,
    const State& b)
{
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;

    const double denominator =
        dx * dx + dy * dy;

    if (denominator < 1e-12)
    {
        return std::hypot(
            p.x - a.x,
            p.y - a.y
        );
    }

    const double t = std::clamp(
        ((p.x - a.x) * dx +
            (p.y - a.y) * dy) / denominator,
        0.0,
        1.0
    );

    return std::hypot(
        p.x - (a.x + t * dx),
        p.y - (a.y + t * dy)
    );
}

// ============================================
// 5. Reference path error
// ============================================

struct PathErrorMetrics
{
    double mean = 0.0;
    double maximum = 0.0;
};

PathErrorMetrics calculatePathError(
    const TrackingResult& tracking,
    const std::vector<State>& reference)
{
    PathErrorMetrics answer;

    if (tracking.samples.empty() ||
        reference.size() < 2)
    {
        return answer;
    }

    for (const auto& sample : tracking.samples)
    {
        double minDistance =
            std::numeric_limits<double>::infinity();

        for (size_t j = 1;
            j < reference.size();
            ++j)
        {
            const double d = pointToSegmentDistance(
                sample.state,
                reference[j - 1],
                reference[j]
            );

            minDistance = std::min(
                minDistance, d
            );
        }

        answer.mean += minDistance;

        answer.maximum = std::max(
            answer.maximum,
            minDistance
        );
    }

    answer.mean /= tracking.samples.size();

    return answer;
}

// ============================================
// 6. Write controller comparison
// ============================================

void writeComparison(
    std::ofstream& file,
    const std::string& name,
    const TrackingResult& result,
    const std::vector<State>& reference)
{
    const auto error = calculatePathError(
        result,
        reference
    );

    double peakSteering = 0.0;
    double steeringVariation = 0.0;

    for (size_t i = 0;
        i < result.samples.size();
        ++i)
    {
        peakSteering = std::max(
            peakSteering,
            std::abs(result.samples[i].steering)
        );

        if (i > 0)
        {
            steeringVariation += std::abs(
                result.samples[i].steering -
                result.samples[i - 1].steering
            );
        }
    }

    const double simulationTime =
        result.samples.empty()
        ? 0.0
        : result.samples.back().time;

    file << name << ","
        << result.success << ","
        << result.collision << ","
        << error.mean << ","
        << error.maximum << ","
        << result.finalPositionError << ","
        << result.finalHeadingError << ","
        << peakSteering * 180.0 / PI << ","
        << steeringVariation * 180.0 / PI << ","
        << simulationTime << "\n";

    std::cout
        << name
        << ": success=" << result.success
        << ", collision=" << result.collision
        << ", meanPathError=" << error.mean
        << ", finalError="
        << result.finalPositionError
        << "\n";
}

// ============================================
// 7. Main function
// ============================================

int main(int argc, char** argv)
{
    // ----------------------------------------
    // Step 1: Output directory
    // ----------------------------------------

    const fs::path output =
        argc >= 2
        ? fs::path(argv[1])
        : fs::current_path() / "results";

    fs::create_directories(output);

    // ----------------------------------------
    // Step 2: Vehicle and map
    // ----------------------------------------

    VehicleModel vehicle(1.2);
    GridMap map;

    vehicleDemo(vehicle, output);

    State start{ 2.0, 2.0, 0.0 };
    State goal{ 17.0, 12.0, PI / 2.0 };

    // ----------------------------------------
    // Step 3: A* planning
    // ----------------------------------------

    std::cout << "\nRunning A*...\n";

    PlanResult astar = planAStar(
        map,
        start,
        goal
    );

    // ----------------------------------------
    // Step 4: Hybrid A* planning
    // ----------------------------------------

    std::cout << "Running Hybrid A*...\n";

    HybridConfig config;

    PlanResult hybrid = planHybridAStar(
        map,
        vehicle,
        start,
        goal,
        config
    );

    savePath(
        output / "astar_path.csv",
        astar
    );

    savePath(
        output / "hybrid_path.csv",
        hybrid
    );

    // ----------------------------------------
    // Step 5: Export map obstacles
    // ----------------------------------------

    std::ofstream obstacleFile(
        output / "obstacles.csv"
    );

    obstacleFile << "xmin,ymin,xmax,ymax\n";

    for (const auto& q : map.obstacles)
    {
        obstacleFile
            << q.xmin << ","
            << q.ymin << ","
            << q.xmax << ","
            << q.ymax << "\n";
    }

    // ----------------------------------------
    // Step 6: Export planning metrics
    // ----------------------------------------

    std::ofstream metrics(
        output / "metrics.csv"
    );

    metrics
        << "algorithm,success,time_ms,"
        << "path_length,max_curvature,"
        << "min_center_clearance,expanded\n";

    auto printMetrics = [&](
        const char* name,
        const PlanResult& result)
        {
            metrics
                << name << ","
                << result.success << ","
                << result.timeMs << ","
                << result.length << ","
                << result.maxCurvature << ","
                << result.minCenterClearance << ","
                << result.expanded << "\n";

            std::cout
                << name
                << ": success=" << result.success
                << ", time=" << result.timeMs
                << "ms, length=" << result.length
                << ", expanded=" << result.expanded
                << "\n";
        };

    printMetrics("AStar", astar);
    printMetrics("HybridAStar", hybrid);

    // ----------------------------------------
    // Step 7: Check planning result
    // ----------------------------------------

    if (!hybrid.success)
    {
        saveVisualization(
            output / "planning_visualization.svg",
            map,
            astar,
            hybrid
        );

        std::cerr
            << "Hybrid A* failed.\n";

        return 2;
    }

    // ----------------------------------------
    // Step 8: Baseline Pure Pursuit
    // ----------------------------------------

    std::cout
        << "\nRunning baseline Pure Pursuit...\n";

    TrackingResult baseline = trackPurePursuit(
        map,
        vehicle,
        hybrid.path,
        1.0,
        1.5
    );

    saveTracking(
        output / "tracking_path.csv",
        baseline
    );

    saveVisualization(
        output / "planning_visualization.svg",
        map,
        astar,
        hybrid,
        &baseline
    );

    // ----------------------------------------
    // Step 9: Original 24 parameter tests
    // ----------------------------------------

    std::cout
        << "\n========== Parameter Sweep ==========\n";

    std::ofstream sweepFile(
        output / "pure_pursuit_benchmark.csv"
    );

    sweepFile
        << "speed,lookahead,success,collision,"
        << "mean_path_error,max_path_error,"
        << "final_position_error,final_heading_error,"
        << "max_steering_deg,simulation_time\n";

    int testNumber = 0;
    int successful = 0;

    for (double speed :
    { 0.5, 1.0, 1.5, 2.0 })
    {
        for (double lookahead :
        { 0.5, 1.0, 1.5, 2.0, 2.5, 3.0 })
        {
            TrackingResult result = trackPurePursuit(
                map,
                vehicle,
                hybrid.path,
                speed,
                lookahead
            );

            const auto error = calculatePathError(
                result,
                hybrid.path
            );

            double maxSteering = 0.0;

            for (const auto& sample : result.samples)
            {
                maxSteering = std::max(
                    maxSteering,
                    std::abs(sample.steering)
                );
            }

            const double simulationTime =
                result.samples.empty()
                ? 0.0
                : result.samples.back().time;

            sweepFile
                << speed << ","
                << lookahead << ","
                << result.success << ","
                << result.collision << ","
                << error.mean << ","
                << error.maximum << ","
                << result.finalPositionError << ","
                << result.finalHeadingError << ","
                << maxSteering * 180.0 / PI << ","
                << simulationTime << "\n";

            successful += result.success;

            std::cout
                << "[" << ++testNumber << "/24] "
                << "speed=" << speed
                << ", lookahead=" << lookahead
                << ", success=" << result.success
                << ", collision=" << result.collision
                << "\n";

            // Save successful comparison case
            if (speed == 1.0 &&
                lookahead == 1.5)
            {
                saveVisualization(
                    output / "tracking_success_1_5.svg",
                    map,
                    astar,
                    hybrid,
                    &result
                );

                saveTracking(
                    output / "tracking_success_1_5.csv",
                    result
                );
            }

            // Save collision comparison case
            if (speed == 1.0 &&
                lookahead == 2.0)
            {
                saveVisualization(
                    output / "tracking_collision_2_0.svg",
                    map,
                    astar,
                    hybrid,
                    &result
                );

                saveTracking(
                    output / "tracking_collision_2_0.csv",
                    result
                );
            }
        }
    }

    sweepFile.close();

    std::cout
        << "Parameter sweep: "
        << successful
        << "/24 successful\n";

    // ----------------------------------------
    // Step 10: Adaptive comparison
    // ----------------------------------------

    std::cout
        << "\n========== Adaptive comparison ==========\n";

    std::ofstream compareFile(
        output / "adaptive_comparison.csv"
    );

    compareFile
        << "method,success,collision,"
        << "mean_path_error,max_path_error,"
        << "final_position_error,final_heading_error,"
        << "max_steering_deg,"
        << "total_steering_change_deg,"
        << "simulation_time\n";

    // Three fixed-lookahead controllers
    for (double lookahead :
    { 1.0, 1.5, 2.0 })
    {
        TrackingResult result = trackPurePursuit(
            map,
            vehicle,
            hybrid.path,
            1.0,
            lookahead
        );

        writeComparison(
            compareFile,
            "fixed_" + std::to_string(lookahead),
            result,
            hybrid.path
        );
    }

    // Four adaptive-lookahead controllers
    for (double gain :
    { 2.0, 4.0, 6.0, 8.0 })
    {
        AdaptiveConfig adaptiveConfig;

        adaptiveConfig.curvatureGain = gain;

        std::vector<double> lookaheadHistory;

        TrackingResult result =
            trackAdaptivePurePursuit(
                map,
                vehicle,
                hybrid.path,
                adaptiveConfig,
                &lookaheadHistory
            );

        writeComparison(
            compareFile,
            "adaptive_gain_" + std::to_string(gain),
            result,
            hybrid.path
        );

        const std::string label =
            std::to_string(static_cast<int>(gain));

        // Export actual trajectory and lookahead
        saveTracking(
            output / ("adaptive_gain_" + label + ".csv"),
            result,
            &lookaheadHistory
        );

        // Export animated SVG
        saveVisualization(
            output / ("adaptive_gain_" + label + ".svg"),
            map,
            astar,
            hybrid,
            &result
        );
    }

    compareFile.close();

    std::cout
        << "Saved adaptive_comparison.csv "
        << "and four adaptive SVG files\n";

    // ============================================
// Multi-scenario benchmark
// ============================================

    runMultiScenarioBenchmark(
        output,
        vehicle
    );

    // ----------------------------------------
    // Step 11: Finish
    // ----------------------------------------

    std::cout
        << "Results directory: "
        << fs::absolute(output)
        << "\n";

    // ============================================
// Steering actuator rate-limit experiment
// ============================================

    robot::steering_test::runBenchmark(
        output,
        vehicle
    );

    // ============================================
// Speed planning and cornering experiment
// ============================================

    robot::speed_test::runBenchmark(
        output,
        vehicle
    );

    return 0;
}
