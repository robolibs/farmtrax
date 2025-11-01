#pragma once

#include "dubins.hpp"
#include <algorithm>
#include <array>
#include <cassert>
#include <concord/concord.hpp>

namespace farmtrax {
    namespace turners {

        enum class ReedsSheppSegmentType {
            LEFT_FORWARD = 'L',
            STRAIGHT_FORWARD = 'S',
            RIGHT_FORWARD = 'R',
            LEFT_BACKWARD = 'l',
            STRAIGHT_BACKWARD = 's',
            RIGHT_BACKWARD = 'r'
        };

        struct ReedsSheppSegment {
            ReedsSheppSegmentType type;
            double length;
            bool forward;

            ReedsSheppSegment(ReedsSheppSegmentType t, double l, bool fwd = true);
        };

        struct ReedsSheppPath {
            std::vector<ReedsSheppSegment> segments;
            std::vector<concord::Pose> waypoints;
            double total_length;
            std::string name;

            ReedsSheppPath();
        };

        class ReedsSheppStateSpace {
          public:
            enum ReedsSheppPathSegmentType { RS_NOP = 0, RS_LEFT = 1, RS_STRAIGHT = 2, RS_RIGHT = 3 };

            static const ReedsSheppPathSegmentType reedsSheppPathType[18][5];

            class ReedsSheppPath {
              public:
                ReedsSheppPath(const ReedsSheppPathSegmentType *type = reedsSheppPathType[0],
                               double t = std::numeric_limits<double>::max(), double u = 0., double v = 0.,
                               double w = 0., double x = 0.)
                    : type_(type), totalLength_(fabs(t) + fabs(u) + fabs(v) + fabs(w) + fabs(x)) {
                    length_[0] = t;
                    length_[1] = u;
                    length_[2] = v;
                    length_[3] = w;
                    length_[4] = x;
                }

                double length() const;

                const ReedsSheppPathSegmentType *type_;
                double length_[5];
                double totalLength_;
            };

            ReedsSheppStateSpace(double turningRadius);

            double distance(double q0[3], double q1[3]) const;
            std::vector<ReedsSheppPathSegmentType> type(double q0[3], double q1[3]) const;
            void sample(double q0[3], double q1[3], double step_size, double &length,
                        std::vector<std::vector<double>> &points) const;
            ReedsSheppPath reedsShepp(double q0[3], double q1[3]) const;
            std::vector<ReedsSheppPath> getAllReedsSheppPaths(double q0[3], double q1[3]) const;
            void interpolate(double q0[3], ReedsSheppPath &path, double seg, double s[3]) const;

          protected:
            double rho_;
        };

        class ReedsShepp : public Dubins {
          public:
            ReedsShepp(double min_turning_radius);

            inline ReedsSheppPath plan_path(const concord::Pose &start, const concord::Pose &end,
                                            double step_size = 0.1) const {
                auto all_paths = get_all_paths(start, end, step_size);

                if (all_paths.empty()) {
                    return ReedsSheppPath();
                }

                auto shortest = std::min_element(
                    all_paths.begin(), all_paths.end(),
                    [](const ReedsSheppPath &a, const ReedsSheppPath &b) { return a.total_length < b.total_length; });

                return *shortest;
            }

            inline std::vector<ReedsSheppPath> get_all_paths(const concord::Pose &start, const concord::Pose &end,
                                                             double step_size = 0.1) const {
                std::vector<ReedsSheppPath> paths;

                // Convert to ReedsSheppStateSpace format
                double q0[3] = {start.point.x, start.point.y, start.angle.yaw};
                double q1[3] = {end.point.x, end.point.y, end.angle.yaw};

                // Get all possible Reeds-Shepp paths using the proper implementation
                auto rs_paths = rs_.getAllReedsSheppPaths(q0, q1);

                // Convert to our format
                for (const auto &rs_path : rs_paths) {
                    if (rs_path.length() < std::numeric_limits<double>::max()) {
                        ReedsSheppPath path;
                        path.total_length = rs_path.length() * radius_;
                        path.name = getPathName(rs_path);

                        // Convert segments
                        for (int i = 0; i < 5; ++i) {
                            if (rs_path.type_[i] == ReedsSheppStateSpace::RS_NOP)
                                break;

                            ReedsSheppSegment segment(ReedsSheppSegmentType::LEFT_FORWARD, 0.0, true);
                            bool forward = rs_path.length_[i] >= 0;

                            switch (rs_path.type_[i]) {
                            case ReedsSheppStateSpace::RS_LEFT:
                                segment.type = forward ? ReedsSheppSegmentType::LEFT_FORWARD
                                                       : ReedsSheppSegmentType::LEFT_BACKWARD;
                                break;
                            case ReedsSheppStateSpace::RS_RIGHT:
                                segment.type = forward ? ReedsSheppSegmentType::RIGHT_FORWARD
                                                       : ReedsSheppSegmentType::RIGHT_BACKWARD;
                                break;
                            case ReedsSheppStateSpace::RS_STRAIGHT:
                                segment.type = forward ? ReedsSheppSegmentType::STRAIGHT_FORWARD
                                                       : ReedsSheppSegmentType::STRAIGHT_BACKWARD;
                                break;
                            case ReedsSheppStateSpace::RS_NOP:
                                break;
                            }
                            segment.length = fabs(rs_path.length_[i]);
                            segment.forward = forward;
                            path.segments.push_back(segment);
                        }

                        // Generate waypoints for this specific path
                        sampleSpecificPath(rs_path, q0, step_size, path.waypoints);

                        paths.push_back(path);
                    }
                }

                return paths;
            }

          private:
            ReedsSheppStateSpace rs_;

            inline void sampleSpecificPath(const ReedsSheppStateSpace::ReedsSheppPath &rs_path, double q0[3],
                                           double step_size, std::vector<concord::Pose> &waypoints) const {
                waypoints.clear();

                double length = rs_path.length() * radius_;

                for (double seg = 0.0; seg <= length; seg += step_size) {
                    double qnew[3] = {};
                    rs_.interpolate(q0, const_cast<ReedsSheppStateSpace::ReedsSheppPath &>(rs_path), seg / radius_,
                                    qnew);

                    concord::Pose pose;
                    pose.point.x = qnew[0];
                    pose.point.y = qnew[1];
                    pose.angle.yaw = qnew[2];
                    waypoints.push_back(pose);
                }
            }

            std::string getPathName(const ReedsSheppStateSpace::ReedsSheppPath &path) const;
        };

    } // namespace turners
} // namespace farmtrax
