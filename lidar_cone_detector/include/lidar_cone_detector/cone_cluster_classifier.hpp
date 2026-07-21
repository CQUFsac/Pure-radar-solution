#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "lidar_cone_detector/geometric_feature_extractor.hpp"

namespace lidar_cone_detector
{

enum class ConeCandidateLevel : std::uint8_t
{
    REJECTED = 0U,
    WEAK = 1U,
    STRONG = 2U
};

enum class ConeRejectReason : std::uint32_t
{
    NONE = 0U,
    INVALID_FEATURES = 1U << 0U,
    TOO_FEW_POINTS = 1U << 1U,
    TOO_SHORT = 1U << 2U,
    TOO_TALL = 1U << 3U,
    TOO_NARROW = 1U << 4U,
    TOO_WIDE = 1U << 5U,
    TOO_SHALLOW = 1U << 6U,
    TOO_DEEP = 1U << 7U,
    BELOW_GROUND = 1U << 8U,
    FLOATING_ABOVE_GROUND = 1U << 9U,
    LOW_SCORE = 1U << 10U
};

struct ConeClassifierConfig
{
    std::array<std::size_t, 4> min_points_by_band{{4U, 3U, 2U, 2U}};
    std::array<std::size_t, 4> reference_points_by_band{{8U, 6U, 4U, 3U}};
    std::array<double, 4> minimum_reliability_by_band{{0.55, 0.50, 0.45, 0.35}};
    std::array<double, 4> maximum_reliability_by_band{{1.00, 0.90, 0.70, 0.55}};

    double hard_min_height = 0.05;
    double preferred_min_height = 0.15;
    double preferred_max_height = 0.50;
    double hard_max_height = 0.75;

    double hard_min_width = 0.02;
    double preferred_min_width = 0.06;
    double preferred_max_width = 0.40;
    double hard_max_width = 0.60;

    double hard_min_depth = 0.005;
    double preferred_min_depth = 0.02;
    double preferred_max_depth = 0.30;
    double hard_max_depth = 0.50;

    bool use_ground_check = false;
    double expected_ground_z = 0.0;
    double hard_min_ground_clearance = -0.10;
    double preferred_min_ground_clearance = -0.03;
    double preferred_max_ground_clearance = 0.10;
    double hard_max_ground_clearance = 0.20;

    bool use_ring_score = true;
    double height_weight = 0.25;
    double width_weight = 0.25;
    double depth_weight = 0.10;
    double ground_weight = 0.15;
    double taper_weight = 0.10;
    double pca_weight = 0.10;
    double ring_weight = 0.05;

    double minimum_structural_score = 0.35;
    double strong_confidence_threshold = 0.70;
    std::size_t first_sparse_band = 2U;
};

struct ConeClassification
{
    std::uint32_t cluster_id = 0U;
    std::size_t band_index = 0U;
    ConeCandidateLevel level = ConeCandidateLevel::REJECTED;
    bool is_candidate = false;

    double height_score = 0.0;
    double width_score = 0.0;
    double depth_score = 0.0;
    double ground_score = 0.0;
    double taper_score = 0.0;
    double pca_score = 0.0;
    double ring_score = 0.0;
    double structural_score = 0.0;
    double measurement_reliability = 0.0;
    double confidence = 0.0;
    double ground_clearance = 0.0;
    std::uint32_t reject_mask = 0U;
};

class ConeClusterClassifier
{
public:
    ConeClusterClassifier();
    explicit ConeClusterClassifier(const ConeClassifierConfig& config);

    ConeClassification classify(const ClusterFeatures& features) const;

private:
    ConeClassifierConfig config_;
};

bool hasRejectReason(
    const ConeClassification& result,
    ConeRejectReason reason);

}  // namespace lidar_cone_detector
