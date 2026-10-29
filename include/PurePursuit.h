
#pragma once

#include "AStar.h"
#include <vector>

namespace robot {

    struct TrackingSample
    {
        State state;

        double time = 0.0;
        double steering = 0.0;
        double crossTrackError = 0.0;
    };

    struct TrackingResult
    {
        bool success = false;
        bool collision = false;

        std::vector<TrackingSample> samples;

        double maxError = 0.0;
        double meanError = 0.0;

        double finalPositionError = 0.0;
        double finalHeadingError = 0.0;

        // New collision diagnostics
        bool hasCollisionState = false;

        State collisionState{};
        State lastSafeState{};

        double collisionTime = 0.0;
        double collisionSteering = 0.0;
    };

    TrackingResult trackPurePursuit(
        const GridMap& map,
        const VehicleModel& vehicle,
        const std::vector<State>& path,
        double speed = 1.0,
        double lookahead = 1.5,
        double maxSteer = PI / 6.0,
        double dt = 0.05,
        int maxSteps = 2000
    );

}
