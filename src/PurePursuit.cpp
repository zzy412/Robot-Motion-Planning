
#include "PurePursuit.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace robot {

    TrackingResult trackPurePursuit(
        const GridMap& map,
        const VehicleModel& vehicle,
        const std::vector<State>& path,
        double speed,
        double lookahead,
        double maxSteer,
        double dt,
        int maxSteps)
    {
        TrackingResult out;

        if (path.size() < 2 ||
            speed <= 0.0 ||
            lookahead <= 0.0 ||
            maxSteer <= 0.0 ||
            dt <= 0.0 ||
            maxSteps <= 0)
        {
            return out;
        }

        State s = path.front();

        size_t nearest = 0;
        double errorSum = 0.0;

        for (int k = 0; k < maxSteps; ++k)
        {
            // 1. Find nearest reference point
            while (nearest + 1 < path.size())
            {
                double d0 = std::hypot(
                    s.x - path[nearest].x,
                    s.y - path[nearest].y
                );

                double d1 = std::hypot(
                    s.x - path[nearest + 1].x,
                    s.y - path[nearest + 1].y
                );

                if (d1 >= d0)
                    break;

                ++nearest;
            }

            // 2. Calculate reference point error
            double error = std::hypot(
                s.x - path[nearest].x,
                s.y - path[nearest].y
            );

            out.maxError = std::max(
                out.maxError,
                error
            );

            errorSum += error;

            // 3. Check goal arrival
            double positionError = std::hypot(
                s.x - path.back().x,
                s.y - path.back().y
            );

            double headingError = std::abs(
                wrapAngle(
                    s.theta - path.back().theta
                )
            );

            if (positionError < 0.4 &&
                headingError < 0.25)
            {
                out.success = true;

                out.samples.push_back({
                    s,
                    k * dt,
                    0.0,
                    error
                    });

                break;
            }

            // 4. Find lookahead target
            size_t target = nearest;

            while (target + 1 < path.size())
            {
                double distance = std::hypot(
                    s.x - path[target].x,
                    s.y - path[target].y
                );

                if (distance >= lookahead)
                    break;

                ++target;
            }

            // 5. Target direction
            double dx = path[target].x - s.x;
            double dy = path[target].y - s.y;

            double alpha = wrapAngle(
                std::atan2(dy, dx) - s.theta
            );

            double ld = std::max(
                0.1,
                std::hypot(dx, dy)
            );

            // 6. Pure Pursuit controller
            double steering = std::atan2(
                2.0 * vehicle.wheelbase()
                * std::sin(alpha),
                ld
            );

            steering = std::clamp(
                steering,
                -maxSteer,
                maxSteer
            );

            // 7. Record current safe state
            out.samples.push_back({
                s,
                k * dt,
                steering,
                error
                });

            // 8. Calculate next vehicle state
            State next = vehicle.step(
                s,
                speed,
                steering,
                dt
            );

            // 9. Check collision and save details
            if (!map.poseFree(next))
            {
                out.collision = true;
                out.hasCollisionState = true;

                // Last accepted state
                out.lastSafeState = s;

                // First rejected candidate state
                out.collisionState = next;

                out.collisionTime = (k + 1) * dt;
                out.collisionSteering = steering;

                std::cout
                    << "Collision diagnostic: "
                    << "t=" << out.collisionTime
                    << " s"
                    << ", lastSafe=("
                    << s.x << "," << s.y << ")"
                    << ", rejected=("
                    << next.x << "," << next.y << ")"
                    << ", heading=" << next.theta
                    << ", steer_deg="
                    << steering * 180.0 / PI
                    << "\n";

                // Do NOT add unsafe pose to samples
                break;
            }

            // 10. Update vehicle state
            s = next;
        }

        // 11. Calculate tracking metrics
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
