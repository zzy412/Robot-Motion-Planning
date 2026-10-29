
#pragma once

#include "MultiScenarioBenchmark.h"

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

    namespace steering_test {

        namespace fs = std::filesystem;

        // ============================================
        // 1. Controller configuration
        // ============================================

        struct Config
        {
            bool adaptive = false;

            double fixedLookahead = 1.0;

            double baseLookahead = 2.0;
            double minLookahead = 0.8;
            double maxLookahead = 2.0;
            double curvatureGain = 4.0;
            double curvatureWindow = 0.6;

            double speed = 1.0;
            double maxSteer = PI / 6.0;

            // Maximum physical steering rate (deg/s).
            // A negative value means unlimited.
            double maxRateDegS = -1.0;

            double dt = 0.05;
            int maxSteps = 2000;
        };

        // ============================================
        // 2. Extended tracking results
        // ============================================

        struct Result
        {
            TrackingResult tracking;

            std::vector<double> commandedSteering;
            std::vector<double> appliedSteering;
            std::vector<double> lookaheadHistory;

            double maxActualRateDegS = 0.0;
            double maxCommandErrorDeg = 0.0;
            double totalActualChangeDeg = 0.0;
        };

        // ============================================
        // 3. Steering rate limiter
        // ============================================

        inline double limitSteeringRate(
            double command,
            double previous,
            double maxRateRadS,
            double dt)
        {
            if (maxRateRadS < 0.0)
            {
                return command;
            }

            const double maxChange = maxRateRadS * dt;

            return std::clamp(
                command,
                previous - maxChange,
                previous + maxChange
            );
        }

        // ============================================
        // 4. Run one controller simulation
        // ============================================

        inline Result simulate(
            const GridMap& map,
            const VehicleModel& vehicle,
            const std::vector<State>& path,
            const Config& cfg)
        {
            Result result;

            if (path.size() < 2 ||
                cfg.speed <= 0.0 ||
                cfg.dt <= 0.0 ||
                cfg.maxSteps <= 0 ||
                cfg.maxSteer <= 0.0 ||
                cfg.fixedLookahead <= 0.0 ||
                cfg.minLookahead <= 0.0 ||
                cfg.maxLookahead < cfg.minLookahead ||
                cfg.curvatureWindow <= 0.0 ||
                cfg.curvatureGain < 0.0)
            {
                return result;
            }

            State state = path.front();

            size_t nearest = 0;

            double previousActualSteer = 0.0;
            double errorSum = 0.0;

            const double maxRateRadS =
                cfg.maxRateDegS < 0.0
                ? -1.0
                : cfg.maxRateDegS * PI / 180.0;

            for (int step = 0; step < cfg.maxSteps; ++step)
            {
                const double now = step * cfg.dt;

                // 1. Find nearest reference point
                while (nearest + 1 < path.size())
                {
                    const double d0 = std::hypot(
                        state.x - path[nearest].x,
                        state.y - path[nearest].y
                    );

                    const double d1 = std::hypot(
                        state.x - path[nearest + 1].x,
                        state.y - path[nearest + 1].y
                    );

                    if (d1 >= d0)
                        break;

                    ++nearest;
                }

                // 2. Discrete reference point error
                const double referenceError = std::hypot(
                    state.x - path[nearest].x,
                    state.y - path[nearest].y
                );

                errorSum += referenceError;

                result.tracking.maxError = std::max(
                    result.tracking.maxError,
                    referenceError
                );

                // 3. Check goal position and heading
                const double positionError = std::hypot(
                    state.x - path.back().x,
                    state.y - path.back().y
                );

                const double headingError = std::abs(
                    wrapAngle(
                        state.theta - path.back().theta
                    )
                );

                if (positionError < 0.4 &&
                    headingError < 0.25)
                {
                    result.tracking.success = true;

                    // Retain the last applied angle.
                    // Do not insert an artificial zero command.
                    result.tracking.samples.push_back({
                        state,
                        now,
                        previousActualSteer,
                        referenceError
                        });

                    result.commandedSteering.push_back(
                        previousActualSteer
                    );

                    result.appliedSteering.push_back(
                        previousActualSteer
                    );

                    result.lookaheadHistory.push_back(
                        std::numeric_limits<double>::quiet_NaN()
                    );

                    break;
                }

                // 4. Calculate lookahead distance
                double lookahead = cfg.fixedLookahead;

                if (cfg.adaptive)
                {
                    const double curvature =
                        localPathCurvature(
                            path,
                            nearest,
                            cfg.curvatureWindow
                        );

                    const double raw =
                        cfg.baseLookahead /
                        (1.0 + cfg.curvatureGain * curvature);

                    lookahead = std::clamp(
                        raw,
                        cfg.minLookahead,
                        cfg.maxLookahead
                    );
                }

                // 5. Find forward target point
                size_t target = nearest;

                while (target + 1 < path.size())
                {
                    const double distance = std::hypot(
                        state.x - path[target].x,
                        state.y - path[target].y
                    );

                    if (distance >= lookahead)
                        break;

                    ++target;
                }

                // 6. Pure Pursuit steering command
                const double dx =
                    path[target].x - state.x;

                const double dy =
                    path[target].y - state.y;

                const double alpha = wrapAngle(
                    std::atan2(dy, dx) - state.theta
                );

                const double targetDistance = std::max(
                    0.1,
                    std::hypot(dx, dy)
                );

                double command = std::atan2(
                    2.0 * vehicle.wheelbase()
                    * std::sin(alpha),
                    targetDistance
                );

                command = std::clamp(
                    command,
                    -cfg.maxSteer,
                    cfg.maxSteer
                );

                // 7. Apply steering actuator rate limit
                const double actualSteer = limitSteeringRate(
                    command,
                    previousActualSteer,
                    maxRateRadS,
                    cfg.dt
                );

                // 8. Controller diagnostics
                const double actualChangeDeg =
                    std::abs(actualSteer -
                        previousActualSteer)
                    * 180.0 / PI;

                const double actualRate =
                    actualChangeDeg / cfg.dt;

                result.maxActualRateDegS = std::max(
                    result.maxActualRateDegS,
                    actualRate
                );

                result.totalActualChangeDeg +=
                    actualChangeDeg;

                const double commandErrorDeg =
                    std::abs(command - actualSteer)
                    * 180.0 / PI;

                result.maxCommandErrorDeg = std::max(
                    result.maxCommandErrorDeg,
                    commandErrorDeg
                );

                // 9. Record the current safe state
                result.tracking.samples.push_back({
                    state,
                    now,
                    actualSteer,
                    referenceError
                    });

                result.commandedSteering.push_back(
                    command
                );

                result.appliedSteering.push_back(
                    actualSteer
                );

                result.lookaheadHistory.push_back(
                    lookahead
                );

                // 10. Vehicle motion with ACTUAL steering
                const State next = vehicle.step(
                    state,
                    cfg.speed,
                    actualSteer,
                    cfg.dt
                );

                // 11. Collision detection
                if (!map.poseFree(next))
                {
                    result.tracking.collision = true;
                    result.tracking.hasCollisionState = true;

                    result.tracking.lastSafeState = state;
                    result.tracking.collisionState = next;

                    result.tracking.collisionTime =
                        now + cfg.dt;

                    result.tracking.collisionSteering =
                        actualSteer;

                    break;
                }

                state = next;
                previousActualSteer = actualSteer;
            }

            // 12. Final metrics
            if (!result.tracking.samples.empty())
            {
                result.tracking.meanError =
                    errorSum /
                    result.tracking.samples.size();
            }

            result.tracking.finalPositionError =
                std::hypot(
                    state.x - path.back().x,
                    state.y - path.back().y
                );

            result.tracking.finalHeadingError =
                std::abs(
                    wrapAngle(
                        state.theta - path.back().theta
                    )
                );

            return result;
        }

        // ============================================
        // 5. Save actuator samples
        // ============================================

        inline void saveActuatorCSV(
            const fs::path& filename,
            const Result& result)
        {
            std::ofstream file(filename);

            file << std::fixed << std::setprecision(8);

            file
                << "time,x,y,theta,"
                << "command_steering_deg,"
                << "actual_steering_deg,"
                << "lookahead\n";

            for (size_t i = 0;
                i < result.tracking.samples.size();
                ++i)
            {
                const auto& q =
                    result.tracking.samples[i];

                file << q.time << ","
                    << q.state.x << ","
                    << q.state.y << ","
                    << q.state.theta << ","
                    << result.commandedSteering[i]
                    * 180.0 / PI << ","
                    << result.appliedSteering[i]
                    * 180.0 / PI << ",";

                if (std::isfinite(result.lookaheadHistory[i]))
                {
                    file << result.lookaheadHistory[i];
                }

                file << "\n";
            }
        }

        // ============================================
        // 6. Describe controller variants
        // ============================================

        struct ControllerCase
        {
            std::string name;
            bool adaptive = false;
            double lookahead = 1.0;
            double gain = 4.0;
        };

        inline std::vector<ControllerCase> makeControllers()
        {
            return {
                {"fixed_1_0", false, 1.0, 0.0},
                {"fixed_1_5", false, 1.5, 0.0},
                {"adaptive_k4", true, 2.0, 4.0},
                {"adaptive_k8", true, 2.0, 8.0}
            };
        }

        // ============================================
        // 7. Five-scenario actuator benchmark
        // ============================================

        inline void runBenchmark(
            const fs::path& output,
            const VehicleModel& vehicle)
        {
            const fs::path folder =
                output / "steering_rate_benchmark";

            fs::create_directories(folder);

            std::ofstream summary(
                folder / "steering_rate_results.csv"
            );

            if (!summary.is_open())
            {
                std::cerr
                    << "Cannot create steering benchmark CSV.\n";
                return;
            }

            summary << std::fixed << std::setprecision(8);

            summary
                << "scenario,controller,rate_limit_deg_s,"
                << "planning_success,success,collision,"
                << "mean_path_error,max_path_error,"
                << "final_position_error,final_heading_error,"
                << "max_actual_rate_deg_s,"
                << "total_actual_change_deg,"
                << "max_command_error_deg,"
                << "simulation_time\n";

            const auto scenes =
                multi_detail::makeScenarios();

            const auto controllers =
                makeControllers();

            const std::vector<double> rates = {
                -1.0, 60.0, 30.0
            };

            int totalRuns = 0;
            int successfulRuns = 0;

            std::cout
                << "\n========== Steering Rate Benchmark ==========\n";

            for (const auto& scene : scenes)
            {
                std::cout
                    << "\nScenario: " << scene.name << "\n";

                const fs::path sceneFolder =
                    folder / scene.name;

                fs::create_directories(sceneFolder);

                if (!scene.map.poseFree(scene.start) ||
                    !scene.map.poseFree(scene.goal))
                {
                    std::cout
                        << "  Invalid start or goal.\n";
                    continue;
                }

                HybridConfig plannerCfg;

                PlanResult hybrid = planHybridAStar(
                    scene.map,
                    vehicle,
                    scene.start,
                    scene.goal,
                    plannerCfg
                );

                if (!hybrid.success)
                {
                    std::cout
                        << "  Hybrid A* failed. Skipping controllers.\n";
                    continue;
                }

                for (const auto& controller : controllers)
                {
                    for (double rate : rates)
                    {
                        Config cfg;

                        cfg.adaptive = controller.adaptive;
                        cfg.fixedLookahead =
                            controller.lookahead;
                        cfg.curvatureGain =
                            controller.gain;
                        cfg.maxRateDegS = rate;

                        // Same speed and time step
                        cfg.speed = 1.0;
                        cfg.dt = 0.05;
                        cfg.maxSteps = 2000;

                        Result result = simulate(
                            scene.map,
                            vehicle,
                            hybrid.path,
                            cfg
                        );

                        const auto pathMetrics =
                            multi_detail::measureTracking(
                                result.tracking,
                                hybrid.path
                            );

                        ++totalRuns;

                        if (result.tracking.success &&
                            !result.tracking.collision)
                        {
                            ++successfulRuns;
                        }

                        const std::string rateName =
                            rate < 0.0
                            ? "unlimited"
                            : "rate_" +
                            std::to_string(
                                static_cast<int>(rate)
                            );

                        const std::string baseName =
                            controller.name + "_" + rateName;

                        const double duration =
                            result.tracking.samples.empty()
                            ? 0.0
                            : result.tracking.samples.back().time;

                        summary
                            << scene.name << ","
                            << controller.name << ","
                            << rate << ","
                            << 1 << ","
                            << result.tracking.success << ","
                            << result.tracking.collision << ","
                            << pathMetrics.meanError << ","
                            << pathMetrics.maxError << ","
                            << result.tracking.finalPositionError
                            << ","
                            << result.tracking.finalHeadingError
                            << ","
                            << result.maxActualRateDegS << ","
                            << result.totalActualChangeDeg << ","
                            << result.maxCommandErrorDeg << ","
                            << duration << "\n";

                        saveActuatorCSV(
                            sceneFolder / (baseName + ".csv"),
                            result
                        );

                        // Save a comparison animation
                        saveVisualization(
                            sceneFolder / (baseName + ".svg"),
                            scene.map,
                            PlanResult{},
                            hybrid,
                            &result.tracking
                        );

                        std::cout
                            << "  " << baseName
                            << ": success="
                            << result.tracking.success
                            << ", collision="
                            << result.tracking.collision
                            << ", maxRate="
                            << result.maxActualRateDegS
                            << " deg/s"
                            << ", meanError="
                            << pathMetrics.meanError
                            << "\n";
                    }
                }
            }

            summary.close();

            std::cout
                << "\nSteering rate benchmark completed.\n"
                << "Controller runs: "
                << totalRuns << "\n"
                << "Successful runs: "
                << successfulRuns << "\n"
                << "Output: "
                << fs::absolute(folder)
                << "\n";
        }

    } // namespace steering_test

} // namespace robot
