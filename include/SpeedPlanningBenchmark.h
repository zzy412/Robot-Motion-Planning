
#pragma once

#include "SteeringRateBenchmark.h"

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
    namespace speed_test {

        namespace fs = std::filesystem;

        // ============================================
        // 1. Speed planning configuration
        // ============================================

        struct Config
        {
            bool adaptiveLookahead = false;
            bool adaptiveSpeed = false;

            double fixedLookahead = 1.0;

            double baseLookahead = 2.0;
            double minLookahead = 0.8;
            double maxLookahead = 2.0;
            double lookaheadGain = 8.0;

            double fixedSpeed = 1.0;
            double minSpeed = 0.35;
            double maxSpeed = 1.0;
            double speedGain = 6.0;

            double maxAcceleration = 0.5;
            double maxDeceleration = 0.8;

            double maxSteer = PI / 6.0;
            double maxSteerRateDegS = 30.0;

            double curvatureWindow = 0.6;
            double anticipationDistance = 1.5;

            double dt = 0.05;
            int maxSteps = 3000;
        };

        // ============================================
        // 2. Simulation result
        // ============================================

        struct Result
        {
            TrackingResult tracking;

            std::vector<double> speedHistory;
            std::vector<double> targetSpeedHistory;

            std::vector<double> lookaheadHistory;
            std::vector<double> curvatureHistory;

            std::vector<double> commandedSteering;
            std::vector<double> actualSteering;

            double maxSteeringRate = 0.0;
            double maxAcceleration = 0.0;
            double maxDeceleration = 0.0;
            double maxSteeringCommandError = 0.0;

            double distanceTravelled = 0.0;
            double movingTime = 0.0;
        };

        // ============================================
        // 3. Estimate upcoming curvature
        // ============================================

        inline double upcomingCurvature(
            const std::vector<State>& path,
            size_t nearest,
            double distance,
            double window)
        {
            double maximum = 0.0;
            double travelled = 0.0;

            size_t j = nearest;

            while (j < path.size())
            {
                maximum = std::max(
                    maximum,
                    localPathCurvature(
                        path, j, window
                    )
                );

                if (travelled >= distance ||
                    j + 1 >= path.size())
                {
                    break;
                }

                travelled += std::hypot(
                    path[j + 1].x - path[j].x,
                    path[j + 1].y - path[j].y
                );

                ++j;
            }

            return maximum;
        }

        // ============================================
        // 4. Speed command from curvature
        // ============================================

        inline double calculateTargetSpeed(
            double curvature,
            const Config& cfg)
        {
            if (!cfg.adaptiveSpeed)
                return cfg.fixedSpeed;

            const double desired =
                cfg.maxSpeed /
                (1.0 + cfg.speedGain * curvature);

            return std::clamp(
                desired,
                cfg.minSpeed,
                cfg.maxSpeed
            );
        }

        // ============================================
        // 5. Rate-limited vehicle simulation
        // ============================================

        inline Result simulate(
            const GridMap& map,
            const VehicleModel& vehicle,
            const std::vector<State>& path,
            const Config& cfg)
        {
            Result out;

            if (path.size() < 2 ||
                cfg.dt <= 0.0 ||
                cfg.maxSteps <= 0 ||
                cfg.fixedLookahead <= 0.0 ||
                cfg.minLookahead <= 0.0 ||
                cfg.maxLookahead < cfg.minLookahead ||
                cfg.minSpeed <= 0.0 ||
                cfg.maxSpeed < cfg.minSpeed ||
                cfg.fixedSpeed <= 0.0 ||
                cfg.speedGain < 0.0 ||
                cfg.lookaheadGain < 0.0 ||
                cfg.maxAcceleration <= 0.0 ||
                cfg.maxDeceleration <= 0.0 ||
                cfg.maxSteer <= 0.0 ||
                cfg.maxSteerRateDegS <= 0.0 ||
                cfg.curvatureWindow <= 0.0 ||
                cfg.anticipationDistance < 0.0)
            {
                return out;
            }

            State state = path.front();

            if (!map.poseFree(state))
                return out;

            size_t nearest = 0;

            double speed = cfg.fixedSpeed;
            double actualSteering = 0.0;
            double errorSum = 0.0;

            const double maxSteerRate =
                cfg.maxSteerRateDegS * PI / 180.0;

            for (int k = 0; k < cfg.maxSteps; ++k)
            {
                const double now = k * cfg.dt;

                // Find nearest path index
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

                const double referenceError = std::hypot(
                    state.x - path[nearest].x,
                    state.y - path[nearest].y
                );

                errorSum += referenceError;

                out.tracking.maxError = std::max(
                    out.tracking.maxError,
                    referenceError
                );

                const double finalPositionError = std::hypot(
                    state.x - path.back().x,
                    state.y - path.back().y
                );

                const double finalHeadingError = std::abs(
                    wrapAngle(
                        state.theta - path.back().theta
                    )
                );

                // Keep the previous experiment's
                // arrival criteria for fair comparison.
                if (finalPositionError < 0.4 &&
                    finalHeadingError < 0.25)
                {
                    out.tracking.success = true;

                    out.tracking.samples.push_back({
                        state,
                        now,
                        actualSteering,
                        referenceError
                        });

                    out.speedHistory.push_back(speed);
                    out.targetSpeedHistory.push_back(speed);
                    out.lookaheadHistory.push_back(
                        std::numeric_limits<double>::quiet_NaN()
                    );
                    out.curvatureHistory.push_back(
                        std::numeric_limits<double>::quiet_NaN()
                    );
                    out.commandedSteering.push_back(
                        actualSteering
                    );
                    out.actualSteering.push_back(
                        actualSteering
                    );

                    break;
                }

                // Look ahead at upcoming curvature
                const double curvature = upcomingCurvature(
                    path,
                    nearest,
                    cfg.anticipationDistance,
                    cfg.curvatureWindow
                );

                // Adaptive lookahead
                double lookahead = cfg.fixedLookahead;

                if (cfg.adaptiveLookahead)
                {
                    const double raw =
                        cfg.baseLookahead /
                        (1.0 + cfg.lookaheadGain * curvature);

                    lookahead = std::clamp(
                        raw,
                        cfg.minLookahead,
                        cfg.maxLookahead
                    );
                }

                // Adaptive target speed
                const double desiredSpeed =
                    calculateTargetSpeed(curvature, cfg);

                // Acceleration and braking limits
                // Only adaptive-speed cases change speed.
                double nextSpeed = speed;

                if (cfg.adaptiveSpeed)
                {
                    const double change = std::clamp(
                        desiredSpeed - speed,
                        -cfg.maxDeceleration * cfg.dt,
                        cfg.maxAcceleration * cfg.dt
                    );

                    nextSpeed = std::clamp(
                        speed + change,
                        cfg.minSpeed,
                        cfg.maxSpeed
                    );
                }

                // Pure Pursuit target index
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

                double steeringCommand = std::atan2(
                    2.0 * vehicle.wheelbase()
                    * std::sin(alpha),
                    targetDistance
                );

                steeringCommand = std::clamp(
                    steeringCommand,
                    -cfg.maxSteer,
                    cfg.maxSteer
                );

                // Physical steering-rate constraint
                const double previousSteering =
                    actualSteering;

                actualSteering = steering_test::limitSteeringRate(
                    steeringCommand,
                    previousSteering,
                    maxSteerRate,
                    cfg.dt
                );

                const double measuredRate =
                    std::abs(
                        actualSteering - previousSteering
                    ) * 180.0 / (PI * cfg.dt);

                out.maxSteeringRate = std::max(
                    out.maxSteeringRate,
                    measuredRate
                );

                out.maxSteeringCommandError = std::max(
                    out.maxSteeringCommandError,
                    std::abs(
                        steeringCommand - actualSteering
                    ) * 180.0 / PI
                );

                const double acceleration =
                    (nextSpeed - speed) / cfg.dt;

                if (acceleration >= 0.0)
                {
                    out.maxAcceleration = std::max(
                        out.maxAcceleration,
                        acceleration
                    );
                }
                else
                {
                    out.maxDeceleration = std::max(
                        out.maxDeceleration,
                        -acceleration
                    );
                }

                speed = nextSpeed;

                // Record current safe pose and controls
                out.tracking.samples.push_back({
                    state,
                    now,
                    actualSteering,
                    referenceError
                    });

                out.speedHistory.push_back(speed);
                out.targetSpeedHistory.push_back(
                    desiredSpeed
                );
                out.lookaheadHistory.push_back(
                    lookahead
                );
                out.curvatureHistory.push_back(
                    curvature
                );
                out.commandedSteering.push_back(
                    steeringCommand
                );
                out.actualSteering.push_back(
                    actualSteering
                );

                // Advance using actual speed and steering
                const State next = vehicle.step(
                    state,
                    speed,
                    actualSteering,
                    cfg.dt
                );

                // Collision check
                if (!map.poseFree(next))
                {
                    out.tracking.collision = true;
                    out.tracking.hasCollisionState = true;

                    out.tracking.lastSafeState = state;
                    out.tracking.collisionState = next;

                    out.tracking.collisionTime =
                        now + cfg.dt;

                    out.tracking.collisionSteering =
                        actualSteering;

                    break;
                }

                out.distanceTravelled += std::hypot(
                    next.x - state.x,
                    next.y - state.y
                );

                out.movingTime += cfg.dt;

                state = next;
            }

            if (!out.tracking.samples.empty())
            {
                out.tracking.meanError =
                    errorSum / out.tracking.samples.size();
            }

            out.tracking.finalPositionError = std::hypot(
                state.x - path.back().x,
                state.y - path.back().y
            );

            out.tracking.finalHeadingError = std::abs(
                wrapAngle(
                    state.theta - path.back().theta
                )
            );

            return out;
        }

        // ============================================
        // 6. Export full speed/steering history
        // ============================================

        inline void saveCSV(
            const fs::path& filename,
            const Result& result)
        {
            std::ofstream file(filename);

            file << std::fixed << std::setprecision(8);

            file
                << "time,x,y,theta,"
                << "actual_speed,target_speed,"
                << "lookahead,curvature,"
                << "command_steering_deg,"
                << "actual_steering_deg\n";

            for (size_t i = 0;
                i < result.tracking.samples.size();
                ++i)
            {
                const auto& sample =
                    result.tracking.samples[i];

                file << sample.time << ","
                    << sample.state.x << ","
                    << sample.state.y << ","
                    << sample.state.theta << ","
                    << result.speedHistory[i] << ","
                    << result.targetSpeedHistory[i] << ",";

                if (std::isfinite(result.lookaheadHistory[i]))
                {
                    file << result.lookaheadHistory[i];
                }

                file << ",";

                if (std::isfinite(result.curvatureHistory[i]))
                {
                    file << result.curvatureHistory[i];
                }

                file << ","
                    << result.commandedSteering[i]
                    * 180.0 / PI
                    << ","
                    << result.actualSteering[i]
                    * 180.0 / PI
                    << "\n";
            }
        }

        // ============================================
        // 7. Main experiment
        // ============================================

        inline void runBenchmark(
            const fs::path& output,
            const VehicleModel& vehicle)
        {
            const fs::path folder =
                output / "speed_planning_benchmark";

            fs::create_directories(folder);

            std::ofstream summary(
                folder / "speed_planning_results.csv"
            );

            summary << std::fixed << std::setprecision(8);

            summary
                << "scenario,method,planning_success,"
                << "success,collision,"
                << "mean_path_error,max_path_error,"
                << "final_position_error,"
                << "final_heading_error,"
                << "simulation_time,distance_travelled,"
                << "max_steering_rate_deg_s,"
                << "max_acceleration,"
                << "max_deceleration,"
                << "max_command_error_deg\n";

            const auto scenarios =
                multi_detail::makeScenarios();

            struct Method
            {
                std::string name;
                bool adaptiveLookahead;
                bool adaptiveSpeed;
            };

            const std::vector<Method> methods = {
                {"baseline", false, false},
                {"adaptive_lookahead", true, false},
                {"adaptive_speed", false, true},
                {"combined", true, true}
            };

            int total = 0;
            int successes = 0;

            std::cout
                << "\n========== Speed Planning Benchmark ==========\n";

            for (const auto& scene : scenarios)
            {
                std::cout
                    << "\nScenario: "
                    << scene.name << "\n";

                if (!scene.map.poseFree(scene.start) ||
                    !scene.map.poseFree(scene.goal))
                {
                    std::cout
                        << "  Invalid start or goal.\n";
                    continue;
                }

                const fs::path sceneFolder =
                    folder / scene.name;

                fs::create_directories(sceneFolder);

                HybridConfig plannerConfig;

                const PlanResult hybrid = planHybridAStar(
                    scene.map,
                    vehicle,
                    scene.start,
                    scene.goal,
                    plannerConfig
                );

                if (!hybrid.success)
                {
                    std::cout
                        << "  Planning failed; skip tracking.\n";
                    continue;
                }

                for (const auto& method : methods)
                {
                    Config cfg;

                    cfg.adaptiveLookahead =
                        method.adaptiveLookahead;

                    cfg.adaptiveSpeed =
                        method.adaptiveSpeed;

                    cfg.fixedLookahead = 1.0;
                    cfg.lookaheadGain = 8.0;

                    cfg.fixedSpeed = 1.0;
                    cfg.minSpeed = 0.35;
                    cfg.maxSpeed = 1.0;
                    cfg.speedGain = 6.0;

                    cfg.maxSteerRateDegS = 30.0;

                    Result result = simulate(
                        scene.map,
                        vehicle,
                        hybrid.path,
                        cfg
                    );

                    const auto metrics =
                        multi_detail::measureTracking(
                            result.tracking,
                            hybrid.path
                        );

                    ++total;

                    if (result.tracking.success &&
                        !result.tracking.collision)
                    {
                        ++successes;
                    }

                    summary
                        << scene.name << ","
                        << method.name << ",1,"
                        << result.tracking.success << ","
                        << result.tracking.collision << ","
                        << metrics.meanError << ","
                        << metrics.maxError << ","
                        << result.tracking.finalPositionError
                        << ","
                        << result.tracking.finalHeadingError
                        << ","
                        << result.movingTime << ","
                        << result.distanceTravelled << ","
                        << result.maxSteeringRate << ","
                        << result.maxAcceleration << ","
                        << result.maxDeceleration << ","
                        << result.maxSteeringCommandError
                        << "\n";

                    saveCSV(
                        sceneFolder /
                        (method.name + ".csv"),
                        result
                    );

                    saveVisualization(
                        sceneFolder /
                        (method.name + ".svg"),
                        scene.map,
                        PlanResult{},
                        hybrid,
                        &result.tracking
                    );

                    std::cout
                        << "  " << method.name
                        << ": success="
                        << result.tracking.success
                        << ", collision="
                        << result.tracking.collision
                        << ", meanError="
                        << metrics.meanError
                        << ", travelTime="
                        << result.movingTime
                        << " s\n";
                }
            }

            summary.close();

            std::cout
                << "\nSpeed planning experiment completed.\n"
                << "Successful runs: "
                << successes << "/" << total
                << "\nOutput: "
                << fs::absolute(folder)
                << "\n";
        }

    } // namespace speed_test
} // namespace robot
