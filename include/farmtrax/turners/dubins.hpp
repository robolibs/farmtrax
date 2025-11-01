#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <concord/concord.hpp>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

namespace farmtrax {
    namespace turners {

        enum class DubinsSegmentType { LEFT = 'L', STRAIGHT = 'S', RIGHT = 'R' };

        struct DubinsSegment {
            DubinsSegmentType type;
            double length;
        };

        struct DubinsPath {
            std::vector<DubinsSegment> segments;
            std::vector<concord::Pose> waypoints;
            double total_length;
            std::string name;

            DubinsPath();
        };

        class DubinsStateSpace {
          public:
            enum DubinsPathSegmentType { DUBINS_LEFT = 0, DUBINS_STRAIGHT = 1, DUBINS_RIGHT = 2 };

            static const DubinsPathSegmentType dubinsPathType[6][3];

            class DubinsPath {
              public:
                DubinsPath(const DubinsPathSegmentType *type = dubinsPathType[0], double t = 0.,
                           double p = std::numeric_limits<double>::max(), double q = 0.)
                    : type_(type) {
                    length_[0] = t;
                    length_[1] = p;
                    length_[2] = q;
                    assert(t >= 0.);
                    assert(p >= 0.);
                    assert(q >= 0.);
                }

                double length() const;

                const DubinsPathSegmentType *type_;
                double length_[3];
            };

            DubinsStateSpace(double turningRadius = 1.0);

            void sample(double q0[3], double q1[3], double step_size, double &length,
                        std::vector<std::vector<double>> &points) const;
            double distance(double q0[3], double q1[3]);
            DubinsPath dubins(double q0[3], double q1[3]) const;

          public:
            void interpolate(double q0[3], DubinsPath &path, double seg, double s[3]) const;

          protected:
            double rho_;
        };

        class Dubins {
          public:
            Dubins(double min_turning_radius);

            DubinsPath plan_path(const concord::Pose &start, const concord::Pose &end, double step_size = 0.1) const;

            std::vector<DubinsPath> get_all_paths(const concord::Pose &start, const concord::Pose &end,
                                                  double step_size = 0.1) const;

          protected:
            double radius_;
            DubinsStateSpace ds_;

            static const double twopi;
            static const double DUBINS_EPS;
            static const double DUBINS_ZERO;

            double mod2pi(double x) const;

            DubinsStateSpace::DubinsPath dubinsLSL(double d, double alpha, double beta) const;

            DubinsStateSpace::DubinsPath dubinsRSR(double d, double alpha, double beta) const;

            DubinsStateSpace::DubinsPath dubinsRSL(double d, double alpha, double beta) const;

            DubinsStateSpace::DubinsPath dubinsLSR(double d, double alpha, double beta) const;

            DubinsStateSpace::DubinsPath dubinsRLR(double d, double alpha, double beta) const;

            DubinsStateSpace::DubinsPath dubinsLRL(double d, double alpha, double beta) const;
        };

    } // namespace turners
} // namespace farmtrax
