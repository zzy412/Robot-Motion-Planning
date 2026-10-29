
#pragma once

#include "VehicleModel.h"
#include "GridMap.h"
#include "AStar.h"
#include "HybridAStar.h"
#include "PurePursuit.h"
#include "AdaptivePurePursuit.h"
#include "Visualization.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace robot {

    namespace multi_detail {

        namespace fs = std::filesystem;

        // ============================================
        // 1. Scenario definition
        // ============================================

        struct Scenario
        {
            std::string name;
            GridMap map;
            State start;
            State goal;
        };

        // ============================================
        // 2. Create five scenarios
        // ============================================

        inline std::vector<Scenario> makeScenarios()
        {
            std::vector<Scenario> scenarios;

            // A: Original map
            {
                Scenario s;
                s.name = "A_original";
                s.start = { 2.0, 2.0, 0.0 };
                s.goal = { 17.0, 12.0, PI / 2.0 };

                // Keep GridMap's original obstacles
                scenarios.push_back(s);
            }

            // B: Open area
            {
                Scenario s;
                s.name = "B_open";
                s.map.obstacles.clear();

                s.start = { 2.0, 2.0, 0.0 };
                s.goal = { 17.0, 16.0, PI / 4.0 };

                scenarios.push_back(s);
            }

            // C: Central obstacle
            {
                Scenario s;
                s.name = "C_central";
                s.map.obstacles.clear();

                s.map.obstacles.push_back({
                    7.0, 5.0, 13.0, 14.0
                    });

                s.start = { 2.0, 2.0, 0.0 };
                s.goal = { 17.0, 16.0, PI / 2.0 };

                scenarios.push_back(s);
            }

            // D: Passage between obstacles
            {
                Scenario s;
                s.name = "D_passage";
                s.map.obstacles.clear();

                s.map.obstacles.push_back({
                    7.0, 0.0, 11.0, 7.5
                    });

                s.map.obstacles.push_back({
                    7.0, 12.5, 11.0, 20.0
                    });

                s.start = { 2.0, 10.0, 0.0 };
                s.goal = { 17.0, 10.0, 0.0 };

                scenarios.push_back(s);
            }

            // E: Alternating obstacles
            {
                Scenario s;
                s.name = "E_zigzag";
                s.map.obstacles.clear();

                s.map.obstacles.push_back({
                    6.0, 7.0, 8.0, 20.0
                    });

                s.map.obstacles.push_back({
                    12.0, 0.0, 14.0, 13.0
                    });

                s.start = { 2.0, 3.0, 0.0 };
                s.goal = { 17.0, 17.0, PI / 2.0 };

                scenarios.push_back(s);
            }

            return scenarios;
        }

        // ============================================
        // 3. Distance from point to segment
        // ============================================

        inline double segmentDistance(
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

            const double projection = std::clamp(
                ((p.x - a.x) * dx +
                    (p.y - a.y) * dy) / denominator,
                0.0,
                1.0
            );

            const double qx = a.x + projection * dx;
            const double qy = a.y + projection * dy;

            return std::hypot(
                p.x - qx,
                p.y - qy
            );
        }

        // ============================================
        // 4. Measure tracking performance
        // ============================================

        struct TrackMetrics
        {
            double meanError = 0.0;
            double maxError = 0.0;

            double maxSteeringDeg = 0.0;
            double totalSteeringChangeDeg = 0.0;
            double maxSteeringRateDegS = 0.0;

            double duration = 0.0;
        };

        inline TrackMetrics measureTracking(
            const TrackingResult& tracking,
            const std::vector<State>& path)
        {
            TrackMetrics m;

            if (tracking.samples.empty() ||
                path.size() < 2)
            {
                return m;
            }

            double totalError = 0.0;

            for (size_t i = 0;
                i < tracking.samples.size();
                ++i)
            {
                const auto& q = tracking.samples[i];

                double nearestDistance =
                    std::numeric_limits<double>::infinity();

                for (size_t j = 1;
                    j < path.size();
                    ++j)
                {
                    nearestDistance = std::min(
                        nearestDistance,
                        segmentDistance(
                            q.state,
                            path[j - 1],
                            path[j]
                        )
                    );
                }

                totalError += nearestDistance;

                m.maxError = std::max(
                    m.maxError,
                    nearestDistance
                );

                m.maxSteeringDeg = std::max(
                    m.maxSteeringDeg,
                    std::abs(q.steering) * 180.0 / PI
                );

                if (i > 0)
                {
                    const auto& previous =
                        tracking.samples[i - 1];

                    const double deltaSteering =
                        std::abs(
                            q.steering -
                            previous.steering
                        ) * 180.0 / PI;

                    m.totalSteeringChangeDeg +=
                        deltaSteering;

                    const double elapsed =
                        q.time - previous.time;

                    if (elapsed > 1e-9)
                    {
                        m.maxSteeringRateDegS =
                            std::max(
                                m.maxSteeringRateDegS,
                                deltaSteering / elapsed
                            );
                    }
                }
            }

            m.meanError =
                totalError / tracking.samples.size();

            m.duration =
                tracking.samples.back().time;

            return m;
        }

        // ============================================
        // 5. Write one controller result
        // ============================================

        inline void writeControllerResult(
            std::ofstream& file,
            const std::string& sceneName,
            const std::string& method,
            const TrackingResult& result,
            const std::vector<State>& reference)
        {
            const TrackMetrics m =
                measureTracking(result, reference);

            file << sceneName << ","
                << method << ","
                << static_cast<int>(result.success) << ","
                << static_cast<int>(result.collision) << ","
                << m.meanError << ","
                << m.maxError << ","
                << result.finalPositionError << ","
                << result.finalHeadingError << ","
                << m.maxSteeringDeg << ","
                << m.totalSteeringChangeDeg << ","
                << m.maxSteeringRateDegS << ","
                << m.duration << "\n";

            std::cout
                << "  " << method
                << ": success=" << result.success
                << ", collision=" << result.collision
                << ", meanError=" << m.meanError
                << ", finalError="
                << result.finalPositionError
                << "\n";
        }

        // ============================================
        // 6. Save tracking samples
        // ============================================

        inline void saveTrackingCSV(
            const fs::path& filename,
            const TrackingResult& result)
        {
            std::ofstream file(filename);

            file << "time,x,y,theta,steering,error\n";

            for (const auto& q : result.samples)
            {
                file << q.time << ","
                    << q.state.x << ","
                    << q.state.y << ","
                    << q.state.theta << ","
                    << q.steering << ","
                    << q.crossTrackError << "\n";
            }
        }

        // ============================================
        // 7. Save planned reference path
        // ============================================

        inline void saveReferenceCSV(
            const fs::path& filename,
            const PlanResult& result)
        {
            std::ofstream file(filename);

            file << "x,y,theta\n";

            for (const auto& p : result.path)
            {
                file << p.x << ","
                    << p.y << ","
                    << p.theta << "\n";
            }
        }

    } // namespace multi_detail

    // ============================================
    // 8. Run multi-scenario benchmark
    // ============================================

    inline void runMultiScenarioBenchmark(
        const std::filesystem::path& output,
        const VehicleModel& vehicle)
    {
        namespace fs = std::filesystem;
        using namespace multi_detail;

        const fs::path folder =
            output / "multi_scenario";

        fs::create_directories(folder);

        std::ofstream scenarioFile(
            folder / "scenario_summary.csv"
        );

        std::ofstream controllerFile(
            folder / "controller_results.csv"
        );

        scenarioFile
            << "scenario,start_valid,goal_valid,"
            << "planning_success,planning_time_ms,"
            << "path_length,expanded\n";

        controllerFile
            << "scenario,method,success,collision,"
            << "mean_path_error,max_path_error,"
            << "final_position_error,final_heading_error,"
            << "max_steering_deg,total_steering_change_deg,"
            << "max_steering_rate_deg_s,simulation_time\n";

        const auto scenarios = makeScenarios();

        int validPlanningCases = 0;
        int controllerRuns = 0;
        int controllerSuccesses = 0;

        std::cout
            << "\n========== Multi-Scenario Benchmark ==========\n";

        for (const auto& scene : scenarios)
        {
            std::cout
                << "\nScenario: " << scene.name << "\n";

            const fs::path sceneFolder =
                folder / scene.name;

            fs::create_directories(sceneFolder);

            const bool startValid =
                scene.map.poseFree(scene.start);

            const bool goalValid =
                scene.map.poseFree(scene.goal);

            if (!startValid || !goalValid)
            {
                std::cout
                    << "Invalid start or goal pose. "
                    << "Skipping planning.\n";

                scenarioFile
                    << scene.name << ","
                    << static_cast<int>(startValid) << ","
                    << static_cast<int>(goalValid)
                    << ",0,0,0,0\n";

                continue;
            }

            // ------------------------------------
            // Hybrid A* reference planning
            // ------------------------------------

            HybridConfig cfg;

            PlanResult hybrid = planHybridAStar(
                scene.map,
                vehicle,
                scene.start,
                scene.goal,
                cfg
            );

            scenarioFile
                << scene.name << ",1,1,"
                << static_cast<int>(hybrid.success) << ","
                << hybrid.timeMs << ","
                << hybrid.length << ","
                << hybrid.expanded << "\n";

            std::cout
                << "  Hybrid A*: success="
                << hybrid.success
                << ", time=" << hybrid.timeMs
                << " ms, length=" << hybrid.length
                << "\n";

            // Save map and planning result
            PlanResult emptyAstar;

            saveVisualization(
                sceneFolder / "reference.svg",
                scene.map,
                emptyAstar,
                hybrid,
                nullptr
            );

            saveReferenceCSV(
                sceneFolder / "reference.csv",
                hybrid
            );

            if (!hybrid.success)
            {
                std::cout
                    << "  Planning failed: "
                    << "controller tests skipped.\n";

                continue;
            }

            ++validPlanningCases;

            // ------------------------------------
            // Fixed Pure Pursuit
            // ------------------------------------

            const std::vector<double> fixedValues = {
                1.0, 1.5, 2.0
            };

            const std::vector<std::string> fixedNames = {
                "fixed_1_0",
                "fixed_1_5",
                "fixed_2_0"
            };

            for (size_t i = 0;
                i < fixedValues.size();
                ++i)
            {
                TrackingResult result =
                    trackPurePursuit(
                        scene.map,
                        vehicle,
                        hybrid.path,
                        1.0,
                        fixedValues[i],
                        PI / 6.0,
                        0.05,
                        2000
                    );

                ++controllerRuns;
                if (result.success && !result.collision)
                    ++controllerSuccesses;

                writeControllerResult(
                    controllerFile,
                    scene.name,
                    fixedNames[i],
                    result,
                    hybrid.path
                );

                saveTrackingCSV(
                    sceneFolder /
                    (fixedNames[i] + ".csv"),
                    result
                );

                saveVisualization(
                    sceneFolder /
                    (fixedNames[i] + ".svg"),
                    scene.map,
                    emptyAstar,
                    hybrid,
                    &result
                );
            }

            // ------------------------------------
            // Adaptive Pure Pursuit
            // ------------------------------------

            const std::vector<double> gains = {
                4.0, 8.0
            };

            const std::vector<std::string> adaptiveNames = {
                "adaptive_k4",
                "adaptive_k8"
            };

            for (size_t i = 0;
                i < gains.size();
                ++i)
            {
                AdaptiveConfig acfg;

                acfg.speed = 1.0;
                acfg.baseLookahead = 2.0;
                acfg.minLookahead = 0.8;
                acfg.maxLookahead = 2.0;
                acfg.curvatureGain = gains[i];

                TrackingResult result =
                    trackAdaptivePurePursuit(
                        scene.map,
                        vehicle,
                        hybrid.path,
                        acfg
                    );

                ++controllerRuns;
                if (result.success && !result.collision)
                    ++controllerSuccesses;

                writeControllerResult(
                    controllerFile,
                    scene.name,
                    adaptiveNames[i],
                    result,
                    hybrid.path
                );

                saveTrackingCSV(
                    sceneFolder /
                    (adaptiveNames[i] + ".csv"),
                    result
                );

                saveVisualization(
                    sceneFolder /
                    (adaptiveNames[i] + ".svg"),
                    scene.map,
                    emptyAstar,
                    hybrid,
                    &result
                );
            }
        }

        scenarioFile.close();
        controllerFile.close();

        std::cout
            << "\n========== Benchmark Summary ==========\n"
            << "Scenarios: "
            << scenarios.size() << "\n"
            << "Hybrid A* successful scenarios: "
            << validPlanningCases << "\n"
            << "Controller runs: "
            << controllerRuns << "\n"
            << "Successful controller runs: "
            << controllerSuccesses << "\n"
            << "Output: "
            << fs::absolute(folder)
            << "\n";
    }

} // namespace robot
