#include "lidar_cone_detector/cone_cluster_classifier.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace lidar_cone_detector
{
namespace
{

double clamp01(double value)
{
    return std::max(0.0, std::min(1.0, value));
}

double rangeScore(
    double value,
    double hard_minimum,
    double preferred_minimum,
    double preferred_maximum,
    double hard_maximum)
{
    if (!std::isfinite(value) || value < hard_minimum || value > hard_maximum)
    {
        return 0.0;
    }
    if (value >= preferred_minimum && value <= preferred_maximum)
    {
        return 1.0;
    }
    if (value < preferred_minimum)
    {
        const double denominator = preferred_minimum - hard_minimum;
        return denominator > 0.0 ?
            clamp01((value - hard_minimum) / denominator) : 0.0;
    }
    const double denominator = hard_maximum - preferred_maximum;
    return denominator > 0.0 ?
        clamp01((hard_maximum - value) / denominator) : 0.0;
}

void addRejectReason(ConeClassification& result, ConeRejectReason reason)
{
    result.reject_mask |= static_cast<std::uint32_t>(reason);
}

void addWeightedScore(
    double score,
    double weight,
    double& earned,
    double& available)
{
    if (weight <= 0.0)
    {
        return;
    }
    earned += clamp01(score) * weight;
    available += weight;
}

std::size_t safeBandIndex(std::size_t band_index)
{
    return std::min<std::size_t>(band_index, 3U);
}

double measurementReliability(
    const ConeClassifierConfig& config,
    std::size_t band_index,
    std::size_t point_count)
{
    const std::size_t index = safeBandIndex(band_index);
    const std::size_t minimum = config.min_points_by_band[index];
    const std::size_t reference = std::max(
        minimum + 1U,
        config.reference_points_by_band[index]);
    const double progress = clamp01(
        static_cast<double>(point_count > minimum ? point_count - minimum : 0U) /
        static_cast<double>(reference - minimum));
    const double low = config.minimum_reliability_by_band[index];
    const double high = config.maximum_reliability_by_band[index];
    return clamp01(low + progress * (high - low));
}

void validateRange(
    double hard_minimum,
    double preferred_minimum,
    double preferred_maximum,
    double hard_maximum,
    const char* name)
{
    if (!(hard_minimum <= preferred_minimum &&
          preferred_minimum <= preferred_maximum &&
          preferred_maximum <= hard_maximum))
    {
        throw std::invalid_argument(std::string("Invalid classifier range: ") + name);
    }
}

}  // namespace

ConeClusterClassifier::ConeClusterClassifier()
    : ConeClusterClassifier(ConeClassifierConfig())
{
}

ConeClusterClassifier::ConeClusterClassifier(const ConeClassifierConfig& config)
    : config_(config)
{
    validateRange(
        config_.hard_min_height,
        config_.preferred_min_height,
        config_.preferred_max_height,
        config_.hard_max_height,
        "height");
    validateRange(
        config_.hard_min_width,
        config_.preferred_min_width,
        config_.preferred_max_width,
        config_.hard_max_width,
        "width");
    validateRange(
        config_.hard_min_depth,
        config_.preferred_min_depth,
        config_.preferred_max_depth,
        config_.hard_max_depth,
        "depth");
}

ConeClassification ConeClusterClassifier::classify(
    const ClusterFeatures& features) const
{
    ConeClassification result;
    result.cluster_id = features.cluster_id;
    result.band_index = features.band_index;

    if (!features.basic_valid || !features.radial_valid ||
        features.point_count == 0U ||
        !std::isfinite(features.height) ||
        !std::isfinite(features.tangential_width) ||
        !std::isfinite(features.radial_depth))
    {
        addRejectReason(result, ConeRejectReason::INVALID_FEATURES);
        return result;
    }

    const std::size_t index = safeBandIndex(features.band_index);
    if (features.point_count < config_.min_points_by_band[index])
    {
        addRejectReason(result, ConeRejectReason::TOO_FEW_POINTS);
    }

    const bool sparse = features.band_index >= config_.first_sparse_band;
    if (!sparse && features.height < config_.hard_min_height)
    {
        addRejectReason(result, ConeRejectReason::TOO_SHORT);
    }
    if (features.height > config_.hard_max_height)
    {
        addRejectReason(result, ConeRejectReason::TOO_TALL);
    }
    if (!sparse && features.tangential_width < config_.hard_min_width)
    {
        addRejectReason(result, ConeRejectReason::TOO_NARROW);
    }
    if (features.tangential_width > config_.hard_max_width)
    {
        addRejectReason(result, ConeRejectReason::TOO_WIDE);
    }
    if (!sparse && features.radial_depth < config_.hard_min_depth)
    {
        addRejectReason(result, ConeRejectReason::TOO_SHALLOW);
    }
    if (features.radial_depth > config_.hard_max_depth)
    {
        addRejectReason(result, ConeRejectReason::TOO_DEEP);
    }

    if (config_.use_ground_check)
    {
        result.ground_clearance = features.min_z - config_.expected_ground_z;
        if (result.ground_clearance < config_.hard_min_ground_clearance)
        {
            addRejectReason(result, ConeRejectReason::BELOW_GROUND);
        }
        if (result.ground_clearance > config_.hard_max_ground_clearance)
        {
            addRejectReason(result, ConeRejectReason::FLOATING_ABOVE_GROUND);
        }
    }

    if (result.reject_mask != 0U)
    {
        return result;
    }

    double earned = 0.0;
    double available = 0.0;
    result.height_score = rangeScore(
        features.height,
        config_.hard_min_height,
        config_.preferred_min_height,
        config_.preferred_max_height,
        config_.hard_max_height);
    result.width_score = rangeScore(
        features.tangential_width,
        config_.hard_min_width,
        config_.preferred_min_width,
        config_.preferred_max_width,
        config_.hard_max_width);
    result.depth_score = rangeScore(
        features.radial_depth,
        config_.hard_min_depth,
        config_.preferred_min_depth,
        config_.preferred_max_depth,
        config_.hard_max_depth);
    addWeightedScore(result.height_score, config_.height_weight, earned, available);
    addWeightedScore(result.width_score, config_.width_weight, earned, available);
    addWeightedScore(result.depth_score, config_.depth_weight, earned, available);

    if (config_.use_ground_check)
    {
        result.ground_score = rangeScore(
            result.ground_clearance,
            config_.hard_min_ground_clearance,
            config_.preferred_min_ground_clearance,
            config_.preferred_max_ground_clearance,
            config_.hard_max_ground_clearance);
        addWeightedScore(result.ground_score, config_.ground_weight, earned, available);
    }

    if (features.slice_valid)
    {
        result.taper_score = rangeScore(
            features.taper_ratio,
            0.0,
            0.0,
            0.80,
            1.50);
        addWeightedScore(result.taper_score, config_.taper_weight, earned, available);
    }

    if (features.pca_valid)
    {
        const double vertical_score = clamp01((features.verticality - 0.25) / 0.75);
        const double non_planar_score = clamp01(1.0 - features.planarity);
        result.pca_score = 0.70 * vertical_score + 0.30 * non_planar_score;
        addWeightedScore(result.pca_score, config_.pca_weight, earned, available);
    }

    if (config_.use_ring_score)
    {
        result.ring_score = clamp01(
            static_cast<double>(features.unique_ring_count) / 3.0);
        addWeightedScore(result.ring_score, config_.ring_weight, earned, available);
    }

    if (available <= 0.0)
    {
        addRejectReason(result, ConeRejectReason::INVALID_FEATURES);
        return result;
    }

    result.structural_score = earned / available;
    result.measurement_reliability = measurementReliability(
        config_,
        features.band_index,
        features.point_count);
    result.confidence = result.structural_score * result.measurement_reliability;

    if (result.structural_score < config_.minimum_structural_score)
    {
        addRejectReason(result, ConeRejectReason::LOW_SCORE);
        return result;
    }

    result.is_candidate = true;
    if (sparse)
    {
        result.level = ConeCandidateLevel::WEAK;
    }
    else if (result.confidence >= config_.strong_confidence_threshold)
    {
        result.level = ConeCandidateLevel::STRONG;
    }
    else
    {
        result.level = ConeCandidateLevel::WEAK;
    }
    return result;
}

bool hasRejectReason(
    const ConeClassification& result,
    ConeRejectReason reason)
{
    return (result.reject_mask & static_cast<std::uint32_t>(reason)) != 0U;
}

}  // namespace lidar_cone_detector
