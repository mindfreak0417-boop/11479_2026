#pragma once

#include <optional>
#include <string>

#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/geometry/Pose2d.h>
#include <frc2/command/SubsystemBase.h>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>

#include "LimelightHelpers.h"

using namespace units::literals;

class VisionSubsystem2 : public frc2::SubsystemBase {
public:
    struct VisionUpdate {
        frc::Pose2d pose;
        units::second_t timestamp{0_s};

        double xyStdDev = 999999.0;
        double rotStdDev = 999999.0;

        int tagCount = 0;
        units::meter_t avgTagDist{0_m};

        units::meter_t translationError{0_m};
        units::degree_t headingError{0_deg};

        bool accepted = false;
        bool suggestSeed = false;
    };

    struct Config {
        std::string limelightName = "limelight";

        // Hard reject thresholds
        units::meter_t maxAcceptTagDistance = 6.5_m;
        units::meter_t maxHardRejectTranslationError = 2.2_m;
        units::degree_t maxHardRejectHeadingErrorSingleTag = 35_deg;
        units::meters_per_second_t maxRejectLinearSpeed = 5.0_mps;
        units::degrees_per_second_t maxRejectAngularSpeed = 720_deg_per_s;

        // Base XY trust
        double baseXYStdDev = 0.07;
        double minXYStdDev = 0.05;
        double maxXYStdDev = 1.50;

        // Distance / tag-count trust shaping
        double distanceScalar = 0.020;
        double oneTagPenalty = 0.10;
        double twoTagPenalty = 0.03;
        double farSingleTagPenalty = 0.05;
        units::meter_t farSingleTagDistance = 4.5_m;

        // Motion trust shaping
        double linearSpeedScalar = 0.025;
        double angularSpeedScalar = 0.0006;

        // Translation disagreement shaping
        units::meter_t mediumTranslationError = 0.45_m;
        units::meter_t largeTranslationError = 0.90_m;
        double mediumTranslationPenalty = 0.08;
        double largeTranslationPenalty = 0.18;

        // Heading disagreement shaping
        units::degree_t mediumHeadingError = 10_deg;
        units::degree_t largeHeadingError = 20_deg;
        double headingErrorScalar = 0.002;
        double mediumHeadingPenalty = 0.04;
        double largeHeadingPenalty = 0.10;

        // Rotation trust
        double defaultRotStdDev = 999999.0;   // almost ignore rotation by default
        double closeMultiTagRotStdDev = 0.30; // only trust a bit in strong conditions
        units::meter_t rotTrustMaxTagDistance = 2.5_m;
        units::degrees_per_second_t rotTrustMaxAngularSpeed = 120_deg_per_s;
        units::degree_t rotTrustMaxHeadingError = 10_deg;

        // Seed suggestion
        units::meter_t seedMinTranslationError = 0.8_m;
        units::meters_per_second_t seedMaxLinearSpeed = 0.10_mps;
        units::degrees_per_second_t seedMaxAngularSpeed = 25_deg_per_s;
        int seedMinTagCount = 2;
        int seedStableFramesRequired = 3;
        units::meter_t seedConsistencyTolerance = 0.20_m;
        units::degree_t seedHeadingTolerance = 12_deg;
        units::degree_t seedMaxHeadingError = 20_deg;
        units::second_t seedCooldown = 1.0_s;
    };

    enum class RejectReason {
        None,
        NoMeasurement,
        NoTags,
        DuplicateTimestamp,
        TagTooFar,
        LinearSpeedTooHigh,
        AngularSpeedTooHigh,
        LargeHeadingErrorSingleTag,
        LargeDisagreementWhileMoving
    };

    VisionSubsystem2();
    explicit VisionSubsystem2(Config config);

    void Update(
        const frc::Pose2d& robotPose,
        units::meters_per_second_t translationSpeed,
        units::degrees_per_second_t angularVelocity
    );

    std::optional<VisionUpdate> GetLatestUpdate() const;

private:
    Config m_cfg;
    std::optional<VisionUpdate> m_latestUpdate;
    units::second_t m_lastVisionTimestamp{-1_s};

    frc::Pose2d m_lastSeedCandidatePose;
    int m_seedStableFrames = 0;
    units::second_t m_lastSeedTime{-999_s};

    bool IsNewTimestamp(units::second_t timestamp) const;

    RejectReason GetRejectReason(
        int tagCount,
        units::meter_t avgTagDist,
        units::meter_t translationError,
        units::degree_t headingError,
        units::meters_per_second_t translationSpeed,
        units::degrees_per_second_t angularVelocity
    ) const;

    double ComputeXYStdDev(
        int tagCount,
        units::meter_t avgTagDist,
        units::meter_t translationError,
        units::degree_t headingError,
        units::meters_per_second_t translationSpeed,
        units::degrees_per_second_t angularVelocity
    ) const;

    double ComputeRotStdDev(
        int tagCount,
        units::meter_t avgTagDist,
        units::degree_t headingError,
        units::degrees_per_second_t angularVelocity
    ) const;

    bool ComputeSeedSuggestion(
        const frc::Pose2d& visionPose,
        units::second_t timestamp,
        units::meter_t translationError,
        units::degree_t headingError,
        int tagCount,
        units::meters_per_second_t translationSpeed,
        units::degrees_per_second_t angularVelocity
    );

    static const char* RejectReasonToString(RejectReason reason);

    void PublishAcceptedTelemetry(const VisionUpdate& update) const;
    void PublishRejectedTelemetry(
        RejectReason reason,
        units::second_t timestamp = 0_s,
        int tagCount = 0,
        units::meter_t avgTagDist = 0_m,
        units::meter_t translationError = 0_m,
        units::degree_t headingError = 0_deg
    ) const;
};