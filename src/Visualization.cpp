
#include "Visualization.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <vector>

namespace robot {

    void saveVisualization(
        const std::filesystem::path& filename,
        const GridMap& map,
        const PlanResult& astar,
        const PlanResult& hybrid,
        const TrackingResult* tracking)
    {
        std::ofstream f(filename);
        if (!f.is_open()) return;

        constexpr double pi = 3.14159265358979323846;

        const double scale = 30.0;
        const double left = 42.0;
        const double top = 42.0;

        const double w = map.width * scale;
        const double h = map.height * scale;

        const double svgW = w + 110.0;
        const double svgH = h + 165.0;

        const double vehicleLength = 1.8;
        const double vehicleWidth = 0.9;

        auto sx = [&](double x) {
            return left + x * scale;
            };

        auto sy = [&](double y) {
            return top + (map.height - y) * scale;
            };

        f << std::fixed << std::setprecision(7);

        // ========================================
        // 1. SVG header
        // ========================================

        f << "<svg xmlns='http://www.w3.org/2000/svg' "
            << "width='" << svgW
            << "' height='" << svgH
            << "' viewBox='0 0 "
            << svgW << " " << svgH << "'>\n";

        f << "<rect width='100%' height='100%' "
            << "fill='white'/>\n";

        f << "<text x='42' y='25' "
            << "font-family='Arial' font-size='18' "
            << "font-weight='bold'>"
            << "Hybrid A* and Pure Pursuit Tracking"
            << "</text>\n";

        // ========================================
        // 2. Map background and grid
        // ========================================

        f << "<rect x='" << left
            << "' y='" << top
            << "' width='" << w
            << "' height='" << h
            << "' fill='#fafafa' stroke='#555'/>\n";

        for (int x = 0;
            x <= static_cast<int>(map.width);
            ++x)
        {
            f << "<line x1='" << sx(x)
                << "' y1='" << top
                << "' x2='" << sx(x)
                << "' y2='" << top + h
                << "' stroke='#e5e7eb'/>\n";
        }

        for (int y = 0;
            y <= static_cast<int>(map.height);
            ++y)
        {
            f << "<line x1='" << left
                << "' y1='" << sy(y)
                << "' x2='" << left + w
                << "' y2='" << sy(y)
                << "' stroke='#e5e7eb'/>\n";
        }

        // ========================================
        // 3. Obstacles
        // ========================================

        for (const auto& b : map.obstacles)
        {
            f << "<rect x='" << sx(b.xmin)
                << "' y='" << sy(b.ymax)
                << "' width='"
                << (b.xmax - b.xmin) * scale
                << "' height='"
                << (b.ymax - b.ymin) * scale
                << "' fill='#777'/>\n";
        }

        // ========================================
        // 4. Draw paths
        // ========================================

        auto drawPath = [&](
            const std::vector<State>& path,
            const char* color,
            const char* dash)
            {
                if (path.empty()) return;

                f << "<polyline fill='none' "
                    << "stroke='" << color
                    << "' stroke-width='3' "
                    << "stroke-linejoin='round'";

                if (*dash)
                {
                    f << " stroke-dasharray='"
                        << dash << "'";
                }

                f << " points='";

                for (const auto& p : path)
                {
                    f << sx(p.x)
                        << "," << sy(p.y) << " ";
                }

                f << "'/>\n";
            };

        // Blue: A*
        drawPath(astar.path, "#2563eb", "6 4");

        // Red: Hybrid A*
        drawPath(hybrid.path, "#dc2626", "");

        // Green: Pure Pursuit
        std::vector<State> actualPath;

        if (tracking)
        {
            for (const auto& sample : tracking->samples)
            {
                actualPath.push_back(sample.state);
            }
        }

        drawPath(actualPath, "#16a34a", "");

        // ========================================
        // 5. Planned vehicle poses
        // ========================================

        if (!hybrid.path.empty())
        {
            const size_t stride = std::max<size_t>(
                1, hybrid.path.size() / 12
            );

            for (size_t i = 0;
                i < hybrid.path.size();
                i += stride)
            {
                const auto& p = hybrid.path[i];

                double angle = -p.theta * 180.0 / pi;

                f << "<rect x='"
                    << -vehicleLength * scale / 2.0
                    << "' y='"
                    << -vehicleWidth * scale / 2.0
                    << "' width='"
                    << vehicleLength * scale
                    << "' height='"
                    << vehicleWidth * scale
                    << "' fill='#dc2626' "
                    << "fill-opacity='0.06' "
                    << "stroke='#dc2626' "
                    << "stroke-opacity='0.25' "
                    << "transform='translate("
                    << sx(p.x) << " "
                    << sy(p.y) << ") rotate("
                    << angle << ")'/>\n";
            }
        }

        // ========================================
        // 6. Start and end markers
        // ========================================

        if (!hybrid.path.empty())
        {
            const auto& start = hybrid.path.front();
            const auto& end = hybrid.path.back();

            f << "<circle cx='" << sx(start.x)
                << "' cy='" << sy(start.y)
                << "' r='6' fill='#16a34a'/>\n";

            f << "<circle cx='" << sx(end.x)
                << "' cy='" << sy(end.y)
                << "' r='6' fill='#a21caf'/>\n";
        }

        if (!actualPath.empty())
        {
            const auto& end = actualPath.back();

            f << "<circle cx='" << sx(end.x)
                << "' cy='" << sy(end.y)
                << "' r='4' fill='#047857' "
                << "stroke='white' stroke-width='1.5'/>\n";
        }

        // ========================================
        // 7. Collision diagnostic markers
        // ========================================

        if (tracking && tracking->hasCollisionState)
        {
            const State& safe = tracking->lastSafeState;
            const State& hit = tracking->collisionState;

            // Dashed segment from safe to rejected
            f << "<line x1='" << sx(safe.x)
                << "' y1='" << sy(safe.y)
                << "' x2='" << sx(hit.x)
                << "' y2='" << sy(hit.y)
                << "' stroke='#f59e0b' "
                << "stroke-width='3' "
                << "stroke-dasharray='3 3'/>\n";

            // Blue marker: last accepted state
            f << "<circle cx='" << sx(safe.x)
                << "' cy='" << sy(safe.y)
                << "' r='6' fill='#2563eb' "
                << "stroke='white' "
                << "stroke-width='2'/>\n";

            // Red transparent rectangle:
            // first rejected vehicle pose
            double angle = -hit.theta * 180.0 / pi;

            f << "<g transform='translate("
                << sx(hit.x) << " " << sy(hit.y)
                << ") rotate(" << angle << ")'>\n";

            f << "<rect x='"
                << -vehicleLength * scale / 2.0
                << "' y='"
                << -vehicleWidth * scale / 2.0
                << "' width='"
                << vehicleLength * scale
                << "' height='"
                << vehicleWidth * scale
                << "' fill='#ef4444' "
                << "fill-opacity='0.18' "
                << "stroke='#b91c1c' "
                << "stroke-width='2'/>\n";

            // Red X marker
            f << "<line x1='-10' y1='-10' "
                << "x2='10' y2='10' "
                << "stroke='#b91c1c' "
                << "stroke-width='3'/>\n";

            f << "<line x1='-10' y1='10' "
                << "x2='10' y2='-10' "
                << "stroke='#b91c1c' "
                << "stroke-width='3'/>\n";

            f << "</g>\n";
        }

        // ========================================
        // 8. Animate actual tracking vehicle
        // ========================================

        if (tracking && tracking->samples.size() >= 2)
        {
            const auto& samples = tracking->samples;
            const size_t n = samples.size();

            double duration =
                samples.back().time -
                samples.front().time;

            if (duration > 1e-9)
            {
                std::vector<double> headings(n);

                headings[0] =
                    samples[0].state.theta;

                for (size_t i = 1; i < n; ++i)
                {
                    double delta =
                        samples[i].state.theta -
                        samples[i - 1].state.theta;

                    delta = std::atan2(
                        std::sin(delta),
                        std::cos(delta)
                    );

                    headings[i] =
                        headings[i - 1] + delta;
                }

                auto writeTimes = [&]()
                    {
                        for (size_t i = 0; i < n; ++i)
                        {
                            if (i > 0) f << ";";

                            f << (samples[i].time -
                                samples.front().time)
                                / duration;
                        }
                    };

                // Outer translation group
                f << "<g id='tracking_vehicle'>\n";

                // Inner rotation group
                f << "<g id='tracking_heading'>\n";

                f << "<rect x='"
                    << -vehicleLength * scale / 2.0
                    << "' y='"
                    << -vehicleWidth * scale / 2.0
                    << "' width='"
                    << vehicleLength * scale
                    << "' height='"
                    << vehicleWidth * scale
                    << "' rx='3' "
                    << "fill='#065f46' "
                    << "stroke='#064e3b' "
                    << "stroke-width='2'/>\n";

                // Vehicle front direction
                f << "<line x1='0' y1='0' "
                    << "x2='"
                    << vehicleLength * scale * 0.4
                    << "' y2='0' "
                    << "stroke='white' "
                    << "stroke-width='3'/>\n";

                f << "<circle cx='"
                    << vehicleLength * scale * 0.4
                    << "' cy='0' r='4' "
                    << "fill='#facc15'/>\n";

                // Rotation animation
                f << "<animateTransform "
                    << "attributeName='transform' "
                    << "type='rotate' "
                    << "dur='" << duration << "s' "
                    << "repeatCount='indefinite' "
                    << "calcMode='linear' "
                    << "values='";

                for (size_t i = 0; i < n; ++i)
                {
                    if (i > 0) f << ";";

                    f << -headings[i] * 180.0 / pi;
                }

                f << "' keyTimes='";
                writeTimes();
                f << "'/>\n";

                f << "</g>\n";

                // Translation animation
                f << "<animateTransform "
                    << "attributeName='transform' "
                    << "type='translate' "
                    << "dur='" << duration << "s' "
                    << "repeatCount='indefinite' "
                    << "calcMode='linear' "
                    << "values='";

                for (size_t i = 0; i < n; ++i)
                {
                    if (i > 0) f << ";";

                    f << sx(samples[i].state.x)
                        << " "
                        << sy(samples[i].state.y);
                }

                f << "' keyTimes='";
                writeTimes();
                f << "'/>\n";

                f << "</g>\n";
            }
        }

        // ========================================
        // 9. Legend and diagnostic information
        // ========================================

        double legendY = top + h + 32.0;

        f << "<text x='42' y='" << legendY
            << "' font-family='Arial' "
            << "font-size='13' fill='#2563eb'>"
            << "A*: blue dashed</text>\n";

        f << "<text x='225' y='" << legendY
            << "' font-family='Arial' "
            << "font-size='13' fill='#dc2626'>"
            << "Hybrid A*: red</text>\n";

        f << "<text x='425' y='" << legendY
            << "' font-family='Arial' "
            << "font-size='13' fill='#16a34a'>"
            << "Pure Pursuit: green</text>\n";

        if (tracking)
        {
            f << "<text x='42' y='"
                << legendY + 25.0
                << "' font-family='Arial' "
                << "font-size='12'>"
                << "Tracking success: "
                << (tracking->success ? "YES" : "NO")
                << "; collision: "
                << (tracking->collision ? "YES" : "NO")
                << "; final error: "
                << tracking->finalPositionError
                << " m</text>\n";
        }

        if (tracking && tracking->hasCollisionState)
        {
            const State& hit = tracking->collisionState;

            f << "<text x='42' y='"
                << legendY + 50.0
                << "' font-family='Arial' "
                << "font-size='12' fill='#b91c1c'>"
                << "First rejected pose: ("
                << hit.x << ", " << hit.y
                << "), time="
                << tracking->collisionTime
                << " s</text>\n";

            f << "<text x='42' y='"
                << legendY + 72.0
                << "' font-family='Arial' "
                << "font-size='12'>"
                << "Blue dot: last safe pose; "
                << "red X: first rejected pose"
                << "</text>\n";
        }
        else
        {
            f << "<text x='42' y='"
                << legendY + 50.0
                << "' font-family='Arial' "
                << "font-size='12'>"
                << "Dark green vehicle replays "
                << "the controller trajectory"
                << "</text>\n";
        }

        // ========================================
        // 10. End SVG
        // ========================================

        f << "</svg>\n";
    }

} // namespace robot
