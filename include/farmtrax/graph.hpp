#pragma once

#include "farmtrax/field.hpp"
#include "farmtrax/utils/utils.hpp"

#include <datapod/datapod.hpp>
#include <graphix/vertex/algorithms/dijkstra.hpp>
#include <graphix/vertex/graph.hpp>

#include <cmath>
#include <limits>
#include <numeric>
#include <unordered_map>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace farmtrax {

    // Graph types using graphix
    using Graph = graphix::vertex::Graph<datapod::Point>;
    using Vertex = graphix::vertex::Graph<datapod::Point>::VertexId;

    /**
     * @brief Represents an AB line with two endpoints
     */
    struct ABLine {
        datapod::Point A;
        datapod::Point B;
        std::string uuid;
        std::size_t line_id;

        ABLine(const datapod::Point &a, const datapod::Point &b, std::string uuid, std::size_t id)
            : A(a), B(b), uuid(uuid), line_id(id) {}

        double length() const { return A.distance_to(B); }

        bool equal(const Swath &swath) { return uuid == swath.uuid; }
    };

    /**
     * @brief Network graph for swath traversal optimization
     *
     * Builds a graph from swaths and provides algorithms for:
     * - Field traversal (agricultural pattern)
     * - Shortest path computation
     */
    class Nety {
        std::vector<ABLine> ab_lines_;
        std::vector<std::shared_ptr<const Swath>> swaths_;
        Graph graph_;
        datapod::PointRTree<std::size_t> endpoint_tree_;
        std::vector<Vertex> vertex_A_; // A endpoints for each line
        std::vector<Vertex> vertex_B_; // B endpoints for each line

      public:
        /**
         * @brief Constructor from vector of const Swaths (filters to only SwathType::Swath)
         */
        explicit Nety(const std::vector<std::shared_ptr<const Swath>> &swaths) {
            // Filter to only include SwathType::Swath
            for (const auto &swath : swaths) {
                if (swath->type == SwathType::Swath) {
                    swaths_.push_back(swath);
                }
            }

            ab_lines_.reserve(swaths_.size());
            for (size_t i = 0; i < swaths_.size(); ++i) {
                ab_lines_.emplace_back(swaths_[i]->line.start, swaths_[i]->line.end, swaths_[i]->uuid, i);
            }
            build_graph();
        }

        /**
         * @brief Constructor from vector of non-const Swaths (filters to only SwathType::Swath)
         */
        explicit Nety(const std::vector<std::shared_ptr<Swath>> &swaths) {
            // Filter to only include SwathType::Swath
            for (const auto &swath : swaths) {
                if (swath->type == SwathType::Swath) {
                    swaths_.push_back(swath); // implicit conversion to const
                }
            }

            ab_lines_.reserve(swaths_.size());
            for (size_t i = 0; i < swaths_.size(); ++i) {
                ab_lines_.emplace_back(swaths_[i]->line.start, swaths_[i]->line.end, swaths_[i]->uuid, i);
            }
            build_graph();
        }

        /**
         * @brief Main traversal function - creates agricultural field pattern and reorders swaths
         */
        void field_traversal(const datapod::Point &start_point) {
            if (ab_lines_.empty())
                return;

            std::vector<Vertex> path;
            std::vector<bool> visited_lines(ab_lines_.size(), false);

            // Store the traversal order and direction information
            std::vector<std::pair<std::size_t, bool>> traversal_order;

            // Find starting line and endpoint
            auto [current_line_id, current_is_A] = find_closest_endpoint(start_point);
            if (current_line_id == SIZE_MAX)
                return;

            while (true) {
                // Mark current line as visited
                visited_lines[current_line_id] = true;

                // Store traversal information with direction
                traversal_order.emplace_back(current_line_id, current_is_A);

                // Add both vertices of current line to path (A->B or B->A)
                Vertex start_vertex = current_is_A ? vertex_A_[current_line_id] : vertex_B_[current_line_id];
                Vertex end_vertex = current_is_A ? vertex_B_[current_line_id] : vertex_A_[current_line_id];

                path.push_back(start_vertex);
                path.push_back(end_vertex);

                // Find next unvisited line using improved spatial clustering
                datapod::Point current_position =
                    current_is_A ? ab_lines_[current_line_id].B : ab_lines_[current_line_id].A;
                auto [next_line_id, next_is_A] = find_next_field_pattern_line(current_position, visited_lines);

                if (next_line_id == SIZE_MAX)
                    break; // No more unvisited lines

                current_line_id = next_line_id;
                current_is_A = next_is_A;
            }

            // Reorder and orient swaths according to traversal
            reorder_swaths(traversal_order);
        }

        void field_traversal(std::shared_ptr<datapod::Point> start_point = nullptr) {
            datapod::Point ptr;
            if (!start_point) {
                ptr = swaths_[0]->line.start;
            } else {
                ptr = *start_point.get();
            }
            field_traversal(ptr);
        }

        /**
         * @brief Calculate shortest path between two points using Dijkstra and reorder swaths
         */
        void shortest_path(const datapod::Point &start, const datapod::Point &goal) {
            auto [start_line, start_is_A] = find_closest_endpoint(start);
            auto [goal_line, goal_is_A] = find_closest_endpoint(goal);

            if (start_line == SIZE_MAX || goal_line == SIZE_MAX)
                return;

            Vertex start_vertex = start_is_A ? vertex_A_[start_line] : vertex_B_[start_line];
            Vertex goal_vertex = goal_is_A ? vertex_A_[goal_line] : vertex_B_[goal_line];

            // Rebuild graph with connection distances included for shortest path calculation
            rebuild_graph_with_connection_distances();

            // Use graphix dijkstra
            auto result = graphix::vertex::algorithms::dijkstra(graph_, start_vertex, goal_vertex);

            if (!result.found)
                return;

            // Convert vertex path to traversal order considering connection costs
            std::vector<std::pair<std::size_t, bool>> traversal_order = get_line_sequence_with_connections(result.path);

            // Reorder and orient swaths according to shortest path
            reorder_swaths(traversal_order);
        }

        /**
         * @brief Overload for shared_ptr
         */
        void shortest_path(std::shared_ptr<datapod::Point> start = nullptr,
                           std::shared_ptr<datapod::Point> goal = nullptr) {
            datapod::Point start_ptr, goal_ptr;
            if (!start) {
                start_ptr = swaths_[0]->line.start;
            } else {
                start_ptr = *start.get();
            }
            if (!goal) {
                goal_ptr = swaths_[0]->line.end;
            } else {
                goal_ptr = *goal.get();
            }
            shortest_path(start_ptr, goal_ptr);
        }

        /**
         * @brief Get total path distance
         */
        double calculate_path_distance(const std::vector<Vertex> &path) const {
            double total_distance = 0.0;
            for (size_t i = 1; i < path.size(); ++i) {
                auto p1 = graph_[path[i - 1]];
                auto p2 = graph_[path[i]];
                total_distance += p1.distance_to(p2);
            }
            return total_distance;
        }

        // Access methods for visualization
        const Graph &get_graph() const { return graph_; }
        const std::vector<ABLine> &get_ab_lines() const { return ab_lines_; }
        const std::vector<std::shared_ptr<const Swath>> &get_swaths() const { return swaths_; }

        /**
         * @brief Get line sequence from vertex path (for visualization)
         */
        std::vector<std::pair<size_t, bool>> get_line_sequence(const std::vector<Vertex> &path) const {
            std::vector<std::pair<size_t, bool>> sequence;

            for (const auto &vertex : path) {
                auto point = graph_[vertex];

                // Find which line and endpoint this vertex represents
                for (size_t i = 0; i < ab_lines_.size(); ++i) {
                    const double eps = 1e-6;
                    if (point.distance_to(ab_lines_[i].A) < eps) {
                        sequence.emplace_back(i, true); // A endpoint
                        break;
                    } else if (point.distance_to(ab_lines_[i].B) < eps) {
                        sequence.emplace_back(i, false); // B endpoint
                        break;
                    }
                }
            }

            return sequence;
        }

        /**
         * @brief Get line sequence from vertex path considering connection costs
         */
        std::vector<std::pair<size_t, bool>> get_line_sequence_with_connections(const std::vector<Vertex> &path) const {
            std::vector<std::pair<size_t, bool>> sequence;

            if (path.empty())
                return sequence;

            // Process path in pairs to identify line traversals
            for (size_t i = 0; i < path.size(); ++i) {
                auto point = graph_[path[i]];

                // Find which line and endpoint this vertex represents
                for (size_t line_idx = 0; line_idx < ab_lines_.size(); ++line_idx) {
                    const double eps = 1e-6;
                    bool is_A_endpoint = point.distance_to(ab_lines_[line_idx].A) < eps;
                    bool is_B_endpoint = point.distance_to(ab_lines_[line_idx].B) < eps;

                    if (is_A_endpoint || is_B_endpoint) {
                        // Check if this is the start of a line traversal
                        if (i + 1 < path.size()) {
                            auto next_point = graph_[path[i + 1]];
                            bool next_is_other_endpoint = false;

                            if (is_A_endpoint) {
                                next_is_other_endpoint = next_point.distance_to(ab_lines_[line_idx].B) < eps;
                            } else {
                                next_is_other_endpoint = next_point.distance_to(ab_lines_[line_idx].A) < eps;
                            }

                            // If next vertex is the other endpoint of the same line, this is a line traversal
                            if (next_is_other_endpoint) {
                                sequence.emplace_back(line_idx, is_A_endpoint);
                                ++i; // Skip the next vertex as it's the end of this line
                                break;
                            }
                        }

                        // If we reach here, this vertex is part of a connection, not a line traversal
                        break;
                    }
                }
            }

            return sequence;
        }

      private:
        void build_graph() {
            if (ab_lines_.empty())
                return;

            // Clear existing data
            graph_.clear();
            endpoint_tree_.clear();
            vertex_A_.clear();
            vertex_B_.clear();

            vertex_A_.resize(ab_lines_.size());
            vertex_B_.resize(ab_lines_.size());

            // Add vertices for all endpoints
            for (size_t i = 0; i < ab_lines_.size(); ++i) {
                const auto &line = ab_lines_[i];

                // Add A endpoint
                vertex_A_[i] = graph_.add_vertex(line.A);
                endpoint_tree_.insert(line.A, i * 2); // Even indices for A endpoints

                // Add B endpoint
                vertex_B_[i] = graph_.add_vertex(line.B);
                endpoint_tree_.insert(line.B, i * 2 + 1); // Odd indices for B endpoints

                // Add edge between A and B (the work line)
                graph_.add_edge(vertex_A_[i], vertex_B_[i], line.length());
            }

            // Add connections between closest endpoints of different lines
            for (size_t i = 0; i < ab_lines_.size(); ++i) {
                // Connect A endpoint to closest endpoints of other lines
                add_nearest_connections(vertex_A_[i], ab_lines_[i].A, i);
                // Connect B endpoint to closest endpoints of other lines
                add_nearest_connections(vertex_B_[i], ab_lines_[i].B, i);
            }
        }

        /**
         * @brief Rebuild graph with accurate connection distances for shortest path calculation
         */
        void rebuild_graph_with_connection_distances() {
            if (ab_lines_.empty())
                return;

            // Clear existing data
            graph_.clear();
            endpoint_tree_.clear();
            vertex_A_.clear();
            vertex_B_.clear();

            vertex_A_.resize(ab_lines_.size());
            vertex_B_.resize(ab_lines_.size());

            // Add vertices for all endpoints
            for (size_t i = 0; i < ab_lines_.size(); ++i) {
                const auto &line = ab_lines_[i];

                // Add A endpoint
                vertex_A_[i] = graph_.add_vertex(line.A);
                endpoint_tree_.insert(line.A, i * 2); // Even indices for A endpoints

                // Add B endpoint
                vertex_B_[i] = graph_.add_vertex(line.B);
                endpoint_tree_.insert(line.B, i * 2 + 1); // Odd indices for B endpoints

                // Add edge between A and B (the work line) - using actual swath length
                graph_.add_edge(vertex_A_[i], vertex_B_[i], line.length());
            }

            // Add connections with accurate connection distances
            for (size_t i = 0; i < ab_lines_.size(); ++i) {
                // Connect A endpoint to closest endpoints of other lines with actual connection distances
                add_connection_distances(vertex_A_[i], ab_lines_[i].A, i);
                // Connect B endpoint to closest endpoints of other lines with actual connection distances
                add_connection_distances(vertex_B_[i], ab_lines_[i].B, i);
            }
        }

        /**
         * @brief Add connections with accurate distance calculations for shortest path
         */
        void add_connection_distances(Vertex vertex, const datapod::Point &point, size_t current_line_id) {
            auto nearest = endpoint_tree_.query_nearest(point, 8);

            for (const auto &near : nearest) {
                size_t endpoint_index = near.data;
                size_t line_id = endpoint_index / 2;

                // Don't connect to same line
                if (line_id == current_line_id)
                    continue;

                // Calculate actual connection distance (Euclidean distance between endpoints)
                double connection_distance = point.distance_to(near.point);

                // Only connect to reasonably close endpoints
                if (connection_distance > 200.0)
                    continue;

                bool is_A = (endpoint_index % 2 == 0);
                Vertex target_vertex = is_A ? vertex_A_[line_id] : vertex_B_[line_id];

                // Use the actual connection distance as edge weight
                graph_.add_edge(vertex, target_vertex, connection_distance);
            }
        }

        void add_nearest_connections(Vertex vertex, const datapod::Point &point, size_t current_line_id) {
            // Use a more conservative approach to avoid excessive cross-field connections
            auto nearest = endpoint_tree_.query_nearest(point, 8);

            // Sort candidates by a combination of distance and field pattern quality
            std::vector<std::pair<double, datapod::PointRTree<std::size_t>::Entry>> scored_candidates;

            for (const auto &near : nearest) {
                size_t endpoint_index = near.data;
                size_t line_id = endpoint_index / 2;

                // Don't connect to same line
                if (line_id == current_line_id)
                    continue;

                double distance = point.distance_to(near.point);

                // Only connect to reasonably close endpoints to reduce cross-field connections
                if (distance > 150.0)
                    continue; // Adjustable threshold

                // Calculate field-aware score
                double score = calculate_connection_score(point, near.point, current_line_id, line_id, distance);
                scored_candidates.emplace_back(score, near);
            }

            // Sort by score (lower is better) and take the best candidates
            std::sort(scored_candidates.begin(), scored_candidates.end(),
                      [](const auto &a, const auto &b) { return a.first < b.first; });

            // Connect to the 3-5 best candidates (reduced from 5 to avoid over-connectivity)
            size_t max_connections = std::min(size_t(4), scored_candidates.size());
            for (size_t i = 0; i < max_connections; ++i) {
                const auto &near = scored_candidates[i].second;
                size_t endpoint_index = near.data;
                size_t line_id = endpoint_index / 2;

                bool is_A = (endpoint_index % 2 == 0);
                Vertex target_vertex = is_A ? vertex_A_[line_id] : vertex_B_[line_id];

                double distance = point.distance_to(near.point);
                graph_.add_edge(vertex, target_vertex, distance);
            }
        }

        /**
         * @brief Calculate a score for connection quality (lower is better)
         */
        double calculate_connection_score(const datapod::Point &from_point, const datapod::Point &to_point,
                                          size_t from_line_id, size_t to_line_id, double distance) const {
            double score = distance; // Base score is distance

            // Penalty for connecting non-parallel lines (usually indicates cross-field connection)
            if (!lines_have_similar_orientation(from_line_id, to_line_id)) {
                score += distance * 0.5; // 50% penalty for non-parallel connections
            }

            // Bonus for maintaining local clustering
            double clustering_bonus = calculate_clustering_bonus(from_point, to_point);
            score -= clustering_bonus;

            return score;
        }

        /**
         * @brief Calculate bonus for connections that maintain local clustering
         */
        double calculate_clustering_bonus(const datapod::Point &from_point, const datapod::Point &to_point) const {
            // Simple heuristic: shorter connections within a reasonable range get a bonus
            double distance = from_point.distance_to(to_point);

            if (distance < 30.0) {
                return 10.0; // Strong bonus for very close connections
            } else if (distance < 60.0) {
                return 5.0; // Moderate bonus for reasonably close connections
            }

            return 0.0; // No bonus for distant connections
        }

        std::pair<size_t, bool> find_closest_endpoint(const datapod::Point &point) const {
            auto nearest = endpoint_tree_.query_nearest(point, 1);

            if (nearest.empty())
                return {SIZE_MAX, false};

            size_t endpoint_index = nearest[0].data;
            size_t line_id = endpoint_index / 2;
            bool is_A = (endpoint_index % 2 == 0);

            return {line_id, is_A};
        }

        std::pair<size_t, bool> find_closest_unvisited_endpoint(const datapod::Point &point,
                                                                const std::vector<bool> &visited_lines) const {
            auto candidates = endpoint_tree_.query_nearest(point, ab_lines_.size() * 2);

            for (const auto &candidate : candidates) {
                size_t endpoint_index = candidate.data;
                size_t line_id = endpoint_index / 2;

                if (!visited_lines[line_id]) {
                    bool is_A = (endpoint_index % 2 == 0);
                    return {line_id, is_A};
                }
            }

            return {SIZE_MAX, false};
        }

        /**
         * @brief Improved field-pattern aware line selection
         */
        std::pair<size_t, bool> find_next_field_pattern_line(const datapod::Point &current_position,
                                                             const std::vector<bool> &visited_lines) const {
            // Strategy: Use expanding search radius with preference for local clusters
            const std::vector<double> search_radii = {15.0, 30.0, 50.0, 100.0, 200.0, 500.0}; // Progressive search

            for (double radius : search_radii) {
                // Query points within radius
                auto candidates = endpoint_tree_.query_radius(current_position, radius);

                // Find the best candidate within this radius
                auto best_candidate = find_best_local_candidate(current_position, candidates, visited_lines);
                if (best_candidate.first != SIZE_MAX) {
                    return best_candidate;
                }
            }

            // Fallback to global search if local search fails
            return find_closest_unvisited_endpoint(current_position, visited_lines);
        }

        /**
         * @brief Find the best candidate within a local area considering field patterns
         */
        std::pair<size_t, bool>
        find_best_local_candidate(const datapod::Point &current_position,
                                  const datapod::Vector<datapod::PointRTree<std::size_t>::Entry> &candidates,
                                  const std::vector<bool> &visited_lines) const {
            double best_score = std::numeric_limits<double>::max();
            std::pair<size_t, bool> best_candidate = {SIZE_MAX, false};

            for (const auto &candidate : candidates) {
                size_t endpoint_index = candidate.data;
                size_t line_id = endpoint_index / 2;

                if (visited_lines[line_id])
                    continue; // Skip visited lines

                bool is_A = (endpoint_index % 2 == 0);
                double distance = current_position.distance_to(candidate.point);

                // Calculate score considering distance and field pattern preferences
                double score = calculate_field_pattern_score(current_position, candidate.point, line_id, distance);

                if (score < best_score) {
                    best_score = score;
                    best_candidate = {line_id, is_A};
                }
            }

            return best_candidate;
        }

        /**
         * @brief Calculate a score that considers both distance and field pattern quality
         */
        double calculate_field_pattern_score(const datapod::Point &current_pos, const datapod::Point &candidate_pos,
                                             size_t candidate_line_id, double distance) const {
            // Base score is distance (shorter is better)
            double score = distance;

            // Penalty for lines that are not roughly aligned with field direction
            double direction_penalty = calculate_direction_penalty(current_pos, candidate_pos, candidate_line_id);
            score += direction_penalty;

            // Bonus for maintaining consistent local patterns
            double pattern_bonus = calculate_pattern_bonus(current_pos, candidate_line_id);
            score -= pattern_bonus;

            return score;
        }

        /**
         * @brief Calculate penalty for lines that break field direction patterns
         */
        double calculate_direction_penalty(const datapod::Point &current_pos, const datapod::Point &candidate_pos,
                                           size_t candidate_line_id) const {
            // Simple heuristic: prefer lines that maintain similar orientation
            const auto &candidate_line = ab_lines_[candidate_line_id];

            // Vector from current position to candidate
            double dx_move = candidate_pos.x - current_pos.x;
            double dy_move = candidate_pos.y - current_pos.y;

            // Vector along the candidate AB line
            double dx_line = candidate_line.B.x - candidate_line.A.x;
            double dy_line = candidate_line.B.y - candidate_line.A.y;

            // Normalize vectors
            double move_length = std::sqrt(dx_move * dx_move + dy_move * dy_move);
            double line_length = std::sqrt(dx_line * dx_line + dy_line * dy_line);

            if (move_length < 1e-6 || line_length < 1e-6)
                return 0.0;

            dx_move /= move_length;
            dy_move /= move_length;
            dx_line /= line_length;
            dy_line /= line_length;

            // Calculate angle between vectors
            double dot_product = dx_move * dx_line + dy_move * dy_line;
            dot_product = std::max(-1.0, std::min(1.0, dot_product)); // Clamp to [-1, 1]
            double angle = std::acos(std::abs(dot_product));          // Use absolute value for perpendicular preference

            // Prefer perpendicular movement to line direction (typical field pattern)
            double perpendicular_preference = std::abs(angle - M_PI / 2.0);
            return perpendicular_preference * 20.0; // Scale the penalty
        }

        /**
         * @brief Calculate bonus for maintaining consistent local field patterns
         */
        double calculate_pattern_bonus(const datapod::Point &current_pos, size_t candidate_line_id) const {
            // Look for nearby parallel lines as an indicator of good field pattern
            const auto &candidate_line = ab_lines_[candidate_line_id];
            double bonus = 0.0;

            // Search for nearby lines with similar orientation
            double search_radius = 50.0;
            auto nearby_points = endpoint_tree_.query_radius(current_pos, search_radius);

            // Count lines with similar orientation
            int similar_orientation_count = 0;
            for (const auto &point : nearby_points) {
                size_t line_id = point.data / 2;
                if (line_id == candidate_line_id)
                    continue;

                if (lines_have_similar_orientation(candidate_line_id, line_id)) {
                    similar_orientation_count++;
                }
            }

            // More parallel lines nearby = better field pattern
            bonus = similar_orientation_count * 5.0;
            return bonus;
        }

        /**
         * @brief Check if two lines have similar orientation (parallel or nearly parallel)
         */
        bool lines_have_similar_orientation(size_t line1_id, size_t line2_id) const {
            const auto &line1 = ab_lines_[line1_id];
            const auto &line2 = ab_lines_[line2_id];

            // Calculate direction vectors
            double dx1 = line1.B.x - line1.A.x;
            double dy1 = line1.B.y - line1.A.y;
            double dx2 = line2.B.x - line2.A.x;
            double dy2 = line2.B.y - line2.A.y;

            // Normalize
            double len1 = std::sqrt(dx1 * dx1 + dy1 * dy1);
            double len2 = std::sqrt(dx2 * dx2 + dy2 * dy2);

            if (len1 < 1e-6 || len2 < 1e-6)
                return false;

            dx1 /= len1;
            dy1 /= len1;
            dx2 /= len2;
            dy2 /= len2;

            // Calculate angle between lines
            double dot_product = dx1 * dx2 + dy1 * dy2;
            dot_product = std::max(-1.0, std::min(1.0, dot_product));
            double angle = std::acos(std::abs(dot_product)); // Use absolute for parallel/anti-parallel

            // Consider lines similar if angle is < 15 degrees
            return angle < (15.0 * M_PI / 180.0);
        }

        /**
         * @brief Reorder and orient swaths according to the traversal order with direction info
         */
        void reorder_swaths(const std::vector<std::pair<std::size_t, bool>> &traversal_order) {
            if (traversal_order.empty())
                return;

            std::vector<std::shared_ptr<const Swath>> reordered_swaths;
            std::vector<ABLine> reordered_ab_lines;

            // Reserve space for both swaths and connections
            reordered_swaths.reserve(traversal_order.size() * 2);
            reordered_ab_lines.reserve(traversal_order.size() * 2);

            // Reorder and orient swaths according to traversal order
            for (std::size_t i = 0; i < traversal_order.size(); ++i) {
                const auto &[swath_index, start_from_A] = traversal_order[i];
                std::shared_ptr<const Swath> swath = swaths_[swath_index];

                // Create a mutable copy to potentially modify direction
                auto m_swath = std::make_shared<Swath>(*swath);

                // If we need to traverse from B to A (not start_from_A), swap the direction
                if (!start_from_A) {
                    m_swath->swapDirection();
                }

                reordered_swaths.push_back(m_swath);

                // Create corresponding AB line with new orientation
                ABLine reordered_line(m_swath->line.start, m_swath->line.end, m_swath->uuid, reordered_ab_lines.size());
                reordered_ab_lines.push_back(reordered_line);

                // Generate connection swath to the next swath (if not the last one)
                if (i < traversal_order.size() - 1) {
                    const auto &[next_swath_index, next_start_from_A] = traversal_order[i + 1];
                    std::shared_ptr<const Swath> next_swath = swaths_[next_swath_index];

                    // Current swath's end point
                    datapod::Point current_end = m_swath->getTail();

                    // Next swath's start point (considering its orientation)
                    datapod::Point next_start = next_start_from_A ? next_swath->getHead() : next_swath->getTail();

                    // Create connection swath from current end to next start
                    auto connection_swath =
                        std::make_shared<Swath>(create_swath(current_end, next_start, SwathType::Connection));

                    reordered_swaths.push_back(connection_swath);

                    // Create corresponding AB line for the connection
                    ABLine connection_line(connection_swath->line.start, connection_swath->line.end,
                                           connection_swath->uuid, reordered_ab_lines.size());
                    reordered_ab_lines.push_back(connection_line);
                }
            }

            // Update the vectors
            swaths_ = std::move(reordered_swaths);
            ab_lines_ = std::move(reordered_ab_lines);

            // Rebuild the graph with the new ordering
            build_graph();
        }
    };

} // namespace farmtrax
