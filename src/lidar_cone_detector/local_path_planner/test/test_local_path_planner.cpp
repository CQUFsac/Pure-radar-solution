#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "local_path_planner/candidate_edge_generator.hpp"
#include "local_path_planner/path_searcher.hpp"
#include "local_path_planner/path_smoother.hpp"
#include "local_path_planner/path_validator.hpp"
#include "local_path_planner/single_side_recovery.hpp"

namespace local_path_planner
{
namespace
{

ConePoint makeCone(std::uint32_t id, double x, double y)
{
    ConePoint cone;
    cone.id = id;
    cone.position.x = x;
    cone.position.y = y;
    cone.confidence = 0.9;
    cone.confirmed = true;
    return cone;
}

TEST(LocalPathPlanner, BuildsStraightCenterPath)
{
    std::vector<ConePoint> cones;
    cones.push_back(makeCone(0U, 2.0, 1.25));
    cones.push_back(makeCone(1U, 2.0, -1.25));
    cones.push_back(makeCone(2U, 4.0, 1.25));
    cones.push_back(makeCone(3U, 4.0, -1.25));
    cones.push_back(makeCone(4U, 6.0, 1.25));
    cones.push_back(makeCone(5U, 6.0, -1.25));
    cones.push_back(makeCone(6U, 8.0, 1.25));
    cones.push_back(makeCone(7U, 8.0, -1.25));

    CandidateEdgeGenerator edge_generator;
    const std::vector<CandidateEdge> edges = edge_generator.generate(cones);
    ASSERT_EQ(4U, edges.size());

    PathSearcher searcher;
    const SearchResult search_result = searcher.search(edges);
    ASSERT_TRUE(search_result.success) << search_result.reason;

    PathSmoother smoother;
    const std::vector<PathPoint> path = smoother.smooth(
        search_result.midpoints, search_result.confidence);
    ASSERT_FALSE(path.empty());

    PathValidator validator;
    std::string reason;
    EXPECT_TRUE(validator.validate(path, cones, reason)) << reason;
    EXPECT_NEAR(0.0, path.back().y, 1.0e-6);
}

TEST(LocalPathPlanner, RejectsSingleSideConeSequence)
{
    std::vector<ConePoint> cones;
    cones.push_back(makeCone(0U, 2.0, 1.25));
    cones.push_back(makeCone(1U, 4.0, 1.25));
    cones.push_back(makeCone(2U, 6.0, 1.25));
    cones.push_back(makeCone(3U, 8.0, 1.25));

    CandidateEdgeGenerator edge_generator;
    EXPECT_TRUE(edge_generator.generate(cones).empty());
}

TEST(LocalPathPlanner, SemanticPairsAreOneToOne)
{
    std::vector<ConePoint> cones;
    cones.push_back(makeCone(0U, 2.0, 1.5));
    cones.push_back(makeCone(1U, 2.0, -1.5));
    cones.push_back(makeCone(2U, 4.0, 1.5));
    cones.push_back(makeCone(3U, 4.0, -1.5));
    cones[0].semantic_class = SEMANTIC_BLUE;
    cones[1].semantic_class = SEMANTIC_YELLOW;
    cones[2].semantic_class = SEMANTIC_BLUE;
    cones[3].semantic_class = SEMANTIC_YELLOW;

    CandidateEdgeGeneratorConfig config;
    config.use_delaunay = false;
    config.fallback_to_pairwise = true;
    config.use_semantic_pairing = true;
    config.use_semantic_boundary_recovery = false;
    config.min_width = 1.0;
    config.max_width = 6.0;
    config.expected_width = 3.0;
    config.max_longitudinal_offset = 3.0;
    config.max_edges_per_cone = 6;
    CandidateEdgeGenerator edge_generator(config);
    const std::vector<CandidateEdge> edges = edge_generator.generate(cones);

    ASSERT_EQ(2U, edges.size());
    std::vector<int> uses(cones.size(), 0);
    for (const CandidateEdge& edge : edges)
    {
        ++uses[edge.first_cone_index];
        ++uses[edge.second_cone_index];
    }
    for (int count : uses)
    {
        EXPECT_EQ(1, count);
    }
}

TEST(LocalPathPlanner, SemanticSearchMayKeepPairingAlternatives)
{
    std::vector<ConePoint> cones;
    cones.push_back(makeCone(0U, 2.0, 1.5));
    cones.push_back(makeCone(1U, 2.0, -1.5));
    cones.push_back(makeCone(2U, 4.0, 1.5));
    cones.push_back(makeCone(3U, 4.0, -1.5));
    cones[0].semantic_class = SEMANTIC_BLUE;
    cones[1].semantic_class = SEMANTIC_YELLOW;
    cones[2].semantic_class = SEMANTIC_BLUE;
    cones[3].semantic_class = SEMANTIC_YELLOW;

    CandidateEdgeGeneratorConfig config;
    config.use_delaunay = false;
    config.fallback_to_pairwise = true;
    config.use_semantic_pairing = true;
    config.enforce_semantic_one_to_one = false;
    config.use_semantic_boundary_recovery = false;
    config.min_width = 1.0;
    config.max_width = 6.0;
    config.expected_width = 3.0;
    config.max_longitudinal_offset = 3.0;
    config.max_edges_per_cone = 6;
    CandidateEdgeGenerator edge_generator(config);

    EXPECT_GT(edge_generator.generate(cones).size(), 2U);
}

TEST(LocalPathPlanner, ReferenceCanStartAVisibleHairpinSideways)
{
    std::vector<CandidateEdge> edges(3U);
    edges[0].midpoint = Point2D{-0.2, 1.0};
    edges[1].midpoint = Point2D{-1.0, 2.0};
    edges[2].midpoint = Point2D{-2.0, 2.5};
    for (std::size_t index = 0U; index < edges.size(); ++index)
    {
        edges[index].cost = 0.1;
        edges[index].confidence = 0.9;
        edges[index].first_cone_id = static_cast<std::uint32_t>(2U * index);
        edges[index].second_cone_id =
            static_cast<std::uint32_t>(2U * index + 1U);
    }

    std::vector<Point2D> reference;
    reference.push_back(Point2D{0.0, 0.0});
    reference.push_back(Point2D{-0.2, 1.0});
    reference.push_back(Point2D{-1.0, 2.0});
    reference.push_back(Point2D{-2.0, 2.5});

    PathSearcherConfig config;
    config.min_midpoints = 3;
    config.max_start_heading = 2.10;
    config.min_start_forward_x = -1.0;
    config.allow_reference_lateral_start = true;
    config.max_turn_angle = 1.40;
    config.max_reference_distance = 0.6;
    PathSearcher searcher(config);
    const SearchResult result = searcher.search(edges, reference);

    EXPECT_TRUE(result.success) << result.reason;
}

TEST(LocalPathPlanner, RecoversCenterFromOneBoundary)
{
    std::vector<Point2D> reference;
    reference.push_back(Point2D{0.0, 0.0});
    reference.push_back(Point2D{2.0, 0.0});
    reference.push_back(Point2D{4.0, 0.0});

    std::vector<ConePoint> cones;
    cones.push_back(makeCone(0U, 4.0, 1.5));
    cones.push_back(makeCone(1U, 6.0, 1.5));

    SingleSideRecovery recovery;
    const SingleSideRecoveryResult result =
        recovery.recover(cones, reference, 0.9);

    ASSERT_TRUE(result.success) << result.reason;
    EXPECT_GE(result.virtual_point_count, 2);
    ASSERT_FALSE(result.centerline.empty());
    EXPECT_GT(result.centerline.back().x, 5.5);
    EXPECT_NEAR(0.0, result.centerline.back().y, 1.0e-6);
}

TEST(LocalPathPlanner, RejectsBackupConeFarFromOldCenter)
{
    std::vector<Point2D> reference;
    reference.push_back(Point2D{0.0, 0.0});
    reference.push_back(Point2D{5.0, 0.0});

    std::vector<ConePoint> cones;
    cones.push_back(makeCone(0U, 3.0, 3.4));

    SingleSideRecovery recovery;
    const SingleSideRecoveryResult result =
        recovery.recover(cones, reference, 0.9);

    EXPECT_FALSE(result.success);
}

TEST(LocalPathPlanner, BuildsGentleCurve)
{
    std::vector<ConePoint> cones;
    cones.push_back(makeCone(0U, 2.0, 1.25));
    cones.push_back(makeCone(1U, 2.0, -1.25));
    cones.push_back(makeCone(2U, 4.0, 1.45));
    cones.push_back(makeCone(3U, 4.0, -1.05));
    cones.push_back(makeCone(4U, 6.0, 1.85));
    cones.push_back(makeCone(5U, 6.0, -0.65));
    cones.push_back(makeCone(6U, 8.0, 2.45));
    cones.push_back(makeCone(7U, 8.0, -0.05));

    CandidateEdgeGenerator edge_generator;
    const std::vector<CandidateEdge> edges = edge_generator.generate(cones);
    ASSERT_GE(edges.size(), 4U);

    PathSearcher searcher;
    const SearchResult search_result = searcher.search(edges);
    ASSERT_TRUE(search_result.success) << search_result.reason;

    PathSmoother smoother;
    const std::vector<PathPoint> path = smoother.smooth(
        search_result.midpoints, search_result.confidence);
    ASSERT_FALSE(path.empty());

    PathValidator validator;
    std::string reason;
    EXPECT_TRUE(validator.validate(path, cones, reason)) << reason;
    EXPECT_GT(path.back().y, 0.5);
}

TEST(LocalPathPlanner, BridgesExactlyOneMissingMidpoint)
{
    std::vector<CandidateEdge> edges(3U);
    edges[0].midpoint = Point2D{2.0, 0.0};
    edges[1].midpoint = Point2D{4.0, 0.2};
    edges[2].midpoint = Point2D{10.0, 1.0};
    for (std::size_t index = 0U; index < edges.size(); ++index)
    {
        edges[index].cost = 0.1;
        edges[index].confidence = 0.9;
        edges[index].first_cone_id =
            static_cast<std::uint32_t>(2U * index);
        edges[index].second_cone_id =
            static_cast<std::uint32_t>(2U * index + 1U);
    }

    PathSearcherConfig config;
    config.min_midpoints = 3;
    config.max_step_distance = 3.5;
    config.max_gap_step_distance = 7.0;
    config.max_gap_bridges = 1;
    config.max_path_length = 14.0;
    PathSearcher searcher(config);
    const SearchResult result = searcher.search(edges);

    ASSERT_TRUE(result.success) << result.reason;
    EXPECT_EQ(1, result.gap_bridges);
    EXPECT_LT(result.confidence, 0.9);
}

TEST(LocalPathPlanner, ReferencePathRejectsRemoteCandidate)
{
    std::vector<ConePoint> cones;
    cones.push_back(makeCone(0U, 3.0, 4.25));
    cones.push_back(makeCone(1U, 3.0, 1.75));

    std::vector<Point2D> reference;
    reference.push_back(Point2D{0.0, 0.0});
    reference.push_back(Point2D{8.0, 0.0});

    CandidateEdgeGeneratorConfig config;
    config.max_reference_distance = 0.8;
    CandidateEdgeGenerator edge_generator(config);
    EXPECT_TRUE(edge_generator.generate(cones, reference).empty());
}

TEST(LocalPathPlanner, SoftReferenceGateKeepsHairpinRecoveryCandidate)
{
    std::vector<ConePoint> cones;
    cones.push_back(makeCone(0U, 3.0, 4.25));
    cones.push_back(makeCone(1U, 3.0, 1.75));

    std::vector<Point2D> reference;
    reference.push_back(Point2D{0.0, 0.0});
    reference.push_back(Point2D{8.0, 0.0});

    CandidateEdgeGeneratorConfig config;
    config.max_reference_distance = 0.8;
    config.hard_reference_gate = false;
    CandidateEdgeGenerator edge_generator(config);
    EXPECT_FALSE(edge_generator.generate(cones, reference).empty());
}

TEST(LocalPathPlanner, MissionHintSelectsRequestedSkidpadBranch)
{
    std::vector<CandidateEdge> edges(5U);
    edges[0].midpoint = Point2D{2.0, 0.0};
    edges[1].midpoint = Point2D{4.0, 1.0};
    edges[2].midpoint = Point2D{6.0, 2.0};
    edges[3].midpoint = Point2D{4.0, -1.0};
    edges[4].midpoint = Point2D{6.0, -2.0};
    for (std::size_t index = 0U; index < edges.size(); ++index)
    {
        edges[index].cost = 0.1;
        edges[index].confidence = 0.9;
        edges[index].first_cone_id =
            static_cast<std::uint32_t>(2U * index);
        edges[index].second_cone_id =
            static_cast<std::uint32_t>(2U * index + 1U);
    }

    PathSearcherConfig config;
    config.min_midpoints = 3;
    config.max_midpoints = 4;
    config.max_path_length = 8.0;
    PathSearcher searcher(config);

    const SearchResult left = searcher.search(
        edges, std::vector<Point2D>(), 1);
    const SearchResult right = searcher.search(
        edges, std::vector<Point2D>(), -1);

    ASSERT_TRUE(left.success) << left.reason;
    ASSERT_TRUE(right.success) << right.reason;
    EXPECT_GT(left.midpoints.back().y, 0.5);
    EXPECT_LT(right.midpoints.back().y, -0.5);
}

TEST(LocalPathPlanner, ValidatorRejectsPathThroughCone)
{
    std::vector<PathPoint> path;
    for (int index = 0; index <= 15; ++index)
    {
        PathPoint point;
        point.x = 0.2 * index;
        point.y = 0.0;
        point.yaw = 0.0;
        point.curvature = 0.0;
        point.confidence = 0.9;
        path.push_back(point);
    }

    std::vector<ConePoint> cones;
    cones.push_back(makeCone(0U, 1.0, 0.1));

    PathValidator validator;
    std::string reason;
    EXPECT_FALSE(validator.validate(path, cones, reason));
    EXPECT_EQ("path is too close to a cone", reason);
}

TEST(LocalPathPlanner, SmootherProducesFiniteGeometry)
{
    std::vector<Point2D> midpoints;
    midpoints.push_back(Point2D{0.0, 0.0});
    midpoints.push_back(Point2D{2.0, 0.0});
    midpoints.push_back(Point2D{4.0, 0.4});
    midpoints.push_back(Point2D{6.0, 1.2});

    PathSmoother smoother;
    const std::vector<PathPoint> path = smoother.smooth(midpoints, 0.8);
    ASSERT_GE(path.size(), 8U);
    for (const PathPoint& point : path)
    {
        EXPECT_TRUE(std::isfinite(point.x));
        EXPECT_TRUE(std::isfinite(point.y));
        EXPECT_TRUE(std::isfinite(point.yaw));
        EXPECT_TRUE(std::isfinite(point.curvature));
    }
}

}  // namespace
}  // namespace local_path_planner

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
