
#pragma once

#include "GridMap.h"
#include "AStar.h"
#include "PurePursuit.h"

#include <filesystem>

namespace robot {

    void saveVisualization(
        const std::filesystem::path& filename,
        const GridMap& map,
        const PlanResult& astar,
        const PlanResult& hybrid,
        const TrackingResult* tracking = nullptr
    );

}
