
#pragma once

#include "PurePursuit.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace robot {

    struct AdaptiveConfig
    {
        double baseLookahead = 2.0;
        double minLookahead = 0.8;
        double maxLookahead = 2.0;

        double curvatureGain = 6.0;
        double curvatureWindow = 0.6;

        double speed = 1.0;
        double maxSteer = PI / 6.0;

        double dt = 0.05;
        int maxSteps = 2000;
    };

    // Estimate local reference-path curvature
    inline double localPathCurvature(
        const std::vector<State>& path,
        size_t at,
        double window)
    {
        size_t a = at;
        size_t b = at;

        double dsA = 0.0;
        double dsB = 0.0;

        while (a > 0 && dsA < window)
        {
            dsA += std::hypot(
                path[a].x - path[a - 1].x,
                path[a].y - path[a - 1].y
            );
            --a;
        }

        while (b + 1 < path.size() && dsB < window)
        {
            dsB += std::hypot(
                path[b + 1].x - path[b].x,
                path[b + 1].y - path[b].y
            );
            ++b;
        }

        const double arc = dsA + dsB;

        if (arc < 1e-8)
            return 0.0;

        const double deltaHeading = wrapAngle(
            path[b].theta - path[a].theta
        );

        return std::abs(deltaHeading) / arc;
    }

    // Adaptive Pure Pursuit simulation
    inline TrackingResult trackAdaptivePurePursuit(
        const GridMap& map,
        const VehicleModel& vehicle,
        const std::vector<State>& path,
        const AdaptiveConfig& cfg,
        std::vector<double>* lookaheadHistory = nullptr)
    {
        TrackingResult out;

        if (lookaheadHistory)
            lookaheadHistory->clear();

        if (path.size() < 2 ||
            cfg.speed <= 0.0 ||
            cfg.dt <= 0.0 ||
            cfg.maxSteer <= 0.0 ||
            cfg.maxSteps <= 0 ||
            cfg.minLookahead <= 0.0 ||
            cfg.maxLookahead < cfg.minLookahead ||
            cfg.curvatureGain < 0.0 ||
            cfg.curvatureWindow <= 0.0)
        {
            return out;
        }

        State s = path.front();

        size_t nearest = 0;
        double errorSum = 0.0;

        for (int k = 0; k < cfg.maxSteps; ++k)
        {
            // Step 1: Find nearest reference point
            while (
                nearest + 1 < path.size() &&
                std::hypot(
                    s.x - path[nearest + 1].x,
                    s.y - path[nearest + 1].y
                ) <
                std::hypot(
                    s.x - path[nearest].x,
                    s.y - path[nearest].y
                )
                )
            {
                ++nearest;
            }

            // Step 2: Calculate discrete point error
            const double error = std::hypot(
                s.x - path[nearest].x,
                s.y - path[nearest].y
            );

            errorSum += error;

            out.maxError = std::max(
                out.maxError,
                error
            );

            // Step 3: Check arrival condition
            const double posError = std::hypot(
                s.x - path.back().x,
                s.y - path.back().y
            );

            const double headingError = std::abs(
                wrapAngle(
                    s.theta - path.back().theta
                )
            );

            if (posError < 0.4 &&
                headingError < 0.25)
            {
                out.success = true;

                out.samples.push_back({
                    s,
                    k * cfg.dt,
                    0.0,
                    error
                    });

                if (lookaheadHistory)
                    lookaheadHistory->push_back(0.0);

                break;
            }

            // Step 4: Estimate local path curvature
            const double curvature = localPathCurvature(
                path,
                nearest,
                cfg.curvatureWindow
            );

            // Step 5: Calculate adaptive lookahead
            const double rawLookahead =
                cfg.baseLookahead /
                (1.0 + cfg.curvatureGain * curvature);

            const double lookahead = std::clamp(
                rawLookahead,
                cfg.minLookahead,
                cfg.maxLookahead
            );

            // Step 6: Select lookahead target
            size_t target = nearest;

            while (target + 1 < path.size())
            {
                const double distance = std::hypot(
                    s.x - path[target].x,
                    s.y - path[target].y
                );

                if (distance >= lookahead)
                    break;

                ++target;
            }

            // Step 7: Pure Pursuit steering control
            const double dx = path[target].x - s.x;
            const double dy = path[target].y - s.y;

            const double alpha = wrapAngle(
                std::atan2(dy, dx) - s.theta
            );

            const double ld = std::max(
                0.1,
                std::hypot(dx, dy)
            );

            const double rawSteering = std::atan2(
                2.0 * vehicle.wheelbase()
                * std::sin(alpha),
                ld
            );

            const double steering = std::clamp(
                rawSteering,
                -cfg.maxSteer,
                cfg.maxSteer
            );

            // Step 8: Record tracking data
            out.samples.push_back({
                s,
                k * cfg.dt,
                steering,
                error
                });

            if (lookaheadHistory)
                lookaheadHistory->push_back(lookahead);

            // Step 9: Simulate vehicle motion
            const State next = vehicle.step(
                s,
                cfg.speed,
                steering,
                cfg.dt
            );

            // Step 10: Collision detection
            if (!map.poseFree(next))
            {
                out.collision = true;
                out.hasCollisionState = true;

                out.lastSafeState = s;
                out.collisionState = next;

                out.collisionTime =
                    (k + 1) * cfg.dt;

                out.collisionSteering = steering;

                break;
            }

            s = next;
        }

        // Step 11: Tracking statistics
        if (!out.samples.empty())
        {
            out.meanError =
                errorSum / out.samples.size();
        }

        out.finalPositionError = std::hypot(
            s.x - path.back().x,
            s.y - path.back().y
        );

        out.finalHeadingError = std::abs(
            wrapAngle(
                s.theta - path.back().theta
            )
        );

        return out;
    }

} // namespace robot
