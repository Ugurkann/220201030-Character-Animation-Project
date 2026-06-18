// Arkheon Simulation Technologies
// Proprietary and Confidential.
// Unauthorized copying of this file, via any medium, is strictly prohibited.
// © Arkheon Simulation Technologies. All rights reserved.

#include "SimCharAnimNathanBalancedGaitPlugin.h"

#include <algorithm>
#include <model/AnimationModel.h>
#include <model/ModelFactoryRegistry.h>
#include <plugin/IModelPluginService.h>
#include <plugin/IPluginServices.h>
#include <plugin/PluginContext.h>

#include <cmath>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace arkheon::sample::simcharanimnathanbalancedgait {
namespace {

constexpr double kTau = 6.28318530717958647692;
constexpr double kDiagnosticLogIntervalSeconds = 1.0;
constexpr const char* kPluginLogPath = "C:/N8RO/sim_char_anim_nathan_balanced_gait.log";

struct EvaluationTraceState {
    int lastTimeBucket = -1;
    int lastSequenceSegment = -1;
    std::size_t lastJointCount = static_cast<std::size_t>(-1);
    std::size_t lastOverrideCount = static_cast<std::size_t>(-1);
    std::string lastActiveAnimationCode;
    bool lastStopped = false;
    bool lastResult = false;
    bool loggedJointIds = false;
};

std::mutex gEvaluationTraceMutex;
std::unordered_map<std::string, EvaluationTraceState> gEvaluationTraceByKey;

[[nodiscard]] bool hasJoint(
    const std::unordered_set<std::string>& availableJointIds,
    const char* jointId) {
    if (!jointId || *jointId == '\0') {
        return false;
    }
    if (availableJointIds.empty()) {
        return true;
    }
    return availableJointIds.find(jointId) != availableJointIds.end();
}

void pushJoint(
    const std::unordered_set<std::string>& availableJointIds,
    arkheon::astsim::AnimationModelOutput& output,
    const char* jointId,
    double xRad,
    double yRad,
    double zRad) {
    if (hasJoint(availableJointIds, jointId)) {
        output.jointOverrides.push_back({jointId, xRad, yRad, zRad});
    }
}

void appendPluginLog(const std::string& message) {
    std::ofstream output(kPluginLogPath, std::ios::app);
    if (!output.is_open()) {
        return;
    }
    output << message << '\n';
}

[[nodiscard]] double smooth01(double value) {
    const double clamped = std::clamp(value, 0.0, 1.0);
    return clamped * clamped * (3.0 - (2.0 * clamped));
}

void resetPluginLog() {
    std::ofstream output(kPluginLogPath, std::ios::trunc);
    if (!output.is_open()) {
        return;
    }
    output << "session_start\n";
}

[[nodiscard]] std::unordered_set<std::string> collectAvailableJointIds(
    const arkheon::astsim::AnimationModelInput& input) {
    std::unordered_set<std::string> availableJointIds;
    availableJointIds.reserve(input.entity.joints.size());
    for (const auto& joint : input.entity.joints) {
        availableJointIds.insert(joint.jointId);
    }
    return availableJointIds;
}

[[nodiscard]] std::string summarizeJointIds(const arkheon::astsim::AnimationModelInput& input) {
    std::ostringstream summary;
    for (std::size_t index = 0; index < input.entity.joints.size(); ++index) {
        if (index > 0) {
            summary << ',';
        }
        summary << input.entity.joints[index].jointId;
    }
    return summary.str();
}

void logEvaluationTrace(
    const char* evaluatorName,
    const arkheon::astsim::AnimationModelInput& input,
    int sequenceSegment,
    const arkheon::astsim::AnimationModelOutput& output,
    bool result) {
    const int timeBucket = static_cast<int>(std::floor(input.simulationTimeSeconds / kDiagnosticLogIntervalSeconds));
    const std::string traceKey =
        (input.entity.entityId.empty() ? std::string("unknown") : input.entity.entityId) +
        "|" + evaluatorName;

    std::string jointsLine;
    bool shouldLogEvaluation = false;
    {
        std::lock_guard<std::mutex> guard(gEvaluationTraceMutex);
        auto& traceState = gEvaluationTraceByKey[traceKey];

        if (!traceState.loggedJointIds) {
            jointsLine =
                "joints evaluator=" + std::string(evaluatorName) +
                " entity=" + input.entity.entityId +
                " names=" + summarizeJointIds(input);
            traceState.loggedJointIds = true;
        }

        if (traceState.lastTimeBucket != timeBucket ||
            traceState.lastSequenceSegment != sequenceSegment ||
            traceState.lastJointCount != input.entity.joints.size() ||
            traceState.lastOverrideCount != output.jointOverrides.size() ||
            traceState.lastActiveAnimationCode != input.entity.activeAnimationCode ||
            traceState.lastStopped != input.entity.animationStopped ||
            traceState.lastResult != result) {
            traceState.lastTimeBucket = timeBucket;
            traceState.lastSequenceSegment = sequenceSegment;
            traceState.lastJointCount = input.entity.joints.size();
            traceState.lastOverrideCount = output.jointOverrides.size();
            traceState.lastActiveAnimationCode = input.entity.activeAnimationCode;
            traceState.lastStopped = input.entity.animationStopped;
            traceState.lastResult = result;
            shouldLogEvaluation = true;
        }
    }

    if (!jointsLine.empty()) {
        appendPluginLog(jointsLine);
    }

    if (!shouldLogEvaluation) {
        return;
    }

    appendPluginLog(
        "evaluate evaluator=" + std::string(evaluatorName) +
        " entity=" + input.entity.entityId +
        " active=" + input.entity.activeAnimationCode +
        " segment=" + std::to_string(sequenceSegment) +
        " stopped=" + (input.entity.animationStopped ? std::string("true") : std::string("false")) +
        " joints=" + std::to_string(input.entity.joints.size()) +
        " overrides=" + std::to_string(output.jointOverrides.size()) +
        " result=" + (result ? std::string("true") : std::string("false")) +
        " simTime=" + std::to_string(input.simulationTimeSeconds) +
        " normTime=" + std::to_string(input.entity.activeAnimationNormalizedTime));
}

[[nodiscard]] bool evaluatePureWalkAnimation(
    const std::unordered_set<std::string>& availableJointIds,
    double simulationTimeSeconds,
    arkheon::astsim::AnimationModelOutput& output) {
    const double cadenceHz = 0.92;
    const double phase = simulationTimeSeconds * cadenceHz * kTau;
    const double stride = std::sin(phase);
    const double leftSwing = std::max(0.0, stride);
    const double rightSwing = std::max(0.0, -stride);
    const double torsoSway = std::sin(phase + (kTau * 0.25));
    const double elbowFlexBase = 0.60;
    const double leftHipPitch = -0.18 + (0.88 * stride);
    const double rightHipPitch = -0.18 - (0.88 * stride);
    const double leftKneePitch = 0.24 + (1.35 * leftSwing) + (0.12 * rightSwing);
    const double rightKneePitch = 0.24 + (1.35 * rightSwing) + (0.12 * leftSwing);
    const double leftAnklePitch = -0.16 - (0.24 * stride) + (0.38 * leftSwing);
    const double rightAnklePitch = -0.16 + (0.24 * stride) + (0.38 * rightSwing);

    output.clearExistingJointOverrides = true;
    output.jointOverrides.clear();

    pushJoint(availableJointIds, output, "leftShoulder", 0.10 + (0.06 * torsoSway), 0.0, -1.30 - (0.20 * stride));
    pushJoint(availableJointIds, output, "rightShoulder", 0.10 - (0.06 * torsoSway), 0.0, 1.30 + (0.20 * stride));
    pushJoint(availableJointIds, output, "leftElbow", elbowFlexBase + (0.20 * rightSwing), 0.0, -0.08);
    pushJoint(availableJointIds, output, "rightElbow", elbowFlexBase + (0.20 * leftSwing), 0.0, 0.08);
    pushJoint(availableJointIds, output, "leftHip", leftHipPitch, 0.08 * torsoSway, 0.08 * stride);
    pushJoint(availableJointIds, output, "rightHip", rightHipPitch, -0.08 * torsoSway, -0.08 * stride);
    pushJoint(availableJointIds, output, "leftKnee", leftKneePitch, 0.0, 0.0);
    pushJoint(availableJointIds, output, "rightKnee", rightKneePitch, 0.0, 0.0);
    pushJoint(availableJointIds, output, "leftAnkle", leftAnklePitch, 0.0, 0.0);
    pushJoint(availableJointIds, output, "rightAnkle", rightAnklePitch, 0.0, 0.0);

    return !output.jointOverrides.empty();
}

[[nodiscard]] bool evaluatePureKneeLiftAnimation(
    const std::unordered_set<std::string>& availableJointIds,
    double simulationTimeSeconds,
    arkheon::astsim::AnimationModelOutput& output) {
    const double phase = simulationTimeSeconds * 0.42 * kTau;
    const double cycle = std::sin(phase);
    const double leftLift = smooth01(std::max(0.0, cycle));
    const double rightLift = smooth01(std::max(0.0, -cycle));
    const double brace = smooth01(0.5 + (0.5 * std::cos(phase)));
    const double torsoSway = std::sin(phase + (kTau * 0.25));
    const double shoulderLift = 1.10 + (0.10 * brace);
    const double elbowBase = 1.18 + (0.08 * brace);
    const double kneeBase = 0.90 + (0.18 * brace);
    const double ankleBase = -0.38 - (0.06 * brace);

    output.clearExistingJointOverrides = true;
    output.jointOverrides.clear();

    // Keep the broad side-arm silhouette from the reference while making all 10 joints visibly animate.
    pushJoint(
        availableJointIds,
        output,
        "leftShoulder",
        shoulderLift + (0.18 * leftLift) - (0.06 * rightLift),
        0.18 * cycle,
        -1.28 + (0.18 * leftLift) - (0.08 * rightLift));
    pushJoint(
        availableJointIds,
        output,
        "rightShoulder",
        shoulderLift + (0.18 * rightLift) - (0.06 * leftLift),
        -0.18 * cycle,
        1.28 + (0.18 * rightLift) - (0.08 * leftLift));
    pushJoint(
        availableJointIds,
        output,
        "leftElbow",
        elbowBase + (0.52 * leftLift),
        0.12 * cycle,
        -0.48 - (0.10 * leftLift));
    pushJoint(
        availableJointIds,
        output,
        "rightElbow",
        elbowBase + (0.52 * rightLift),
        -0.12 * cycle,
        0.48 + (0.10 * rightLift));
    pushJoint(
        availableJointIds,
        output,
        "leftHip",
        -0.28 + (1.18 * leftLift) - (0.20 * rightLift),
        0.14 * cycle,
        0.18 * torsoSway);
    pushJoint(
        availableJointIds,
        output,
        "rightHip",
        -0.28 + (1.18 * rightLift) - (0.20 * leftLift),
        -0.14 * cycle,
        -0.18 * torsoSway);
    pushJoint(
        availableJointIds,
        output,
        "leftKnee",
        kneeBase + (0.96 * leftLift) + (0.14 * rightLift),
        0.0,
        0.0);
    pushJoint(
        availableJointIds,
        output,
        "rightKnee",
        kneeBase + (0.96 * rightLift) + (0.14 * leftLift),
        0.0,
        0.0);
    pushJoint(
        availableJointIds,
        output,
        "leftAnkle",
        ankleBase + (0.34 * leftLift),
        0.04 * cycle,
        0.0);
    pushJoint(
        availableJointIds,
        output,
        "rightAnkle",
        ankleBase + (0.34 * rightLift),
        -0.04 * cycle,
        0.0);

    return !output.jointOverrides.empty();
}

[[nodiscard]] bool evaluatePurePushAnimation(
    const std::unordered_set<std::string>& availableJointIds,
    double simulationTimeSeconds,
    arkheon::astsim::AnimationModelOutput& output) {
    const double phase = simulationTimeSeconds * 0.18 * kTau;
    const double pulse = std::sin(phase);
    const double brace = 0.5 + (0.5 * pulse);

    output.clearExistingJointOverrides = true;
    output.jointOverrides.clear();

    pushJoint(availableJointIds, output, "leftShoulder", 0.78 + (0.10 * brace), 0.02, -0.68);
    pushJoint(availableJointIds, output, "rightShoulder", 0.78 + (0.10 * brace), -0.02, 0.68);
    pushJoint(availableJointIds, output, "leftElbow", 1.00 - (0.10 * brace), 0.0, -0.08);
    pushJoint(availableJointIds, output, "rightElbow", 1.00 - (0.10 * brace), 0.0, 0.08);
    pushJoint(availableJointIds, output, "leftHip", -0.46 - (0.06 * pulse), 0.0, -0.08);
    pushJoint(availableJointIds, output, "rightHip", -0.46 - (0.06 * pulse), 0.0, 0.08);
    pushJoint(availableJointIds, output, "leftKnee", 1.18 + (0.12 * brace), 0.0, 0.0);
    pushJoint(availableJointIds, output, "rightKnee", 1.18 + (0.12 * brace), 0.0, 0.0);
    pushJoint(availableJointIds, output, "leftAnkle", -0.18 + (0.03 * pulse), 0.0, 0.0);
    pushJoint(availableJointIds, output, "rightAnkle", -0.18 + (0.03 * pulse), 0.0, 0.0);

    return !output.jointOverrides.empty();
}

[[nodiscard]] bool evaluatePureClimbAnimation(
    const std::unordered_set<std::string>& availableJointIds,
    double simulationTimeSeconds,
    arkheon::astsim::AnimationModelOutput& output) {
    const double phase = simulationTimeSeconds * 0.42 * kTau;
    const double cycle = 0.5 + (0.5 * std::sin(phase));
    const double leftLead = cycle;
    const double rightLead = 1.0 - cycle;
    const double torsoPulse = std::sin(phase + (kTau * 0.25));

    output.clearExistingJointOverrides = true;
    output.jointOverrides.clear();

    pushJoint(availableJointIds, output, "leftShoulder", 0.95 + (0.62 * leftLead), -0.04 * torsoPulse, -1.05 - (0.30 * leftLead));
    pushJoint(availableJointIds, output, "rightShoulder", 0.95 + (0.62 * rightLead), 0.04 * torsoPulse, 1.05 + (0.30 * rightLead));
    pushJoint(availableJointIds, output, "leftElbow", 0.96 + (0.34 * leftLead), 0.0, -0.08);
    pushJoint(availableJointIds, output, "rightElbow", 0.96 + (0.34 * rightLead), 0.0, 0.08);
    pushJoint(availableJointIds, output, "leftHip", -0.10 + (1.28 * leftLead) - (0.20 * rightLead), 0.0, -0.06);
    pushJoint(availableJointIds, output, "rightHip", -0.10 + (1.28 * rightLead) - (0.20 * leftLead), 0.0, 0.06);
    pushJoint(availableJointIds, output, "leftKnee", 0.32 + (1.52 * leftLead), 0.0, 0.0);
    pushJoint(availableJointIds, output, "rightKnee", 0.32 + (1.52 * rightLead), 0.0, 0.0);
    pushJoint(availableJointIds, output, "leftAnkle", -0.10 + (0.42 * leftLead), 0.0, 0.0);
    pushJoint(availableJointIds, output, "rightAnkle", -0.10 + (0.42 * rightLead), 0.0, 0.0);

    return !output.jointOverrides.empty();
}

[[nodiscard]] bool evaluatePureIdleStoppedAnimation(
    const std::unordered_set<std::string>& availableJointIds,
    arkheon::astsim::AnimationModelOutput& output) {
    output.clearExistingJointOverrides = true;
    output.jointOverrides.clear();

    pushJoint(availableJointIds, output, "leftShoulder", 0.82, 0.0, -0.18);
    pushJoint(availableJointIds, output, "rightShoulder", 0.82, 0.0, 0.18);
    pushJoint(availableJointIds, output, "leftElbow", 0.42, 0.0, -0.06);
    pushJoint(availableJointIds, output, "rightElbow", 0.42, 0.0, 0.06);
    pushJoint(availableJointIds, output, "leftHip", -0.12, 0.0, -0.04);
    pushJoint(availableJointIds, output, "rightHip", -0.12, 0.0, 0.04);
    pushJoint(availableJointIds, output, "leftKnee", 0.22, 0.0, 0.0);
    pushJoint(availableJointIds, output, "rightKnee", 0.22, 0.0, 0.0);
    pushJoint(availableJointIds, output, "leftAnkle", -0.10, 0.0, 0.0);
    pushJoint(availableJointIds, output, "rightAnkle", -0.10, 0.0, 0.0);

    return !output.jointOverrides.empty();
}

[[nodiscard]] bool evaluateWalkAnimation(
    const arkheon::astsim::AnimationModelInput& input,
    arkheon::astsim::AnimationModelOutput& output) {
    const auto availableJointIds = collectAvailableJointIds(input);
    const bool result = evaluatePureKneeLiftAnimation(availableJointIds, input.simulationTimeSeconds, output);
    logEvaluationTrace("WalkEvaluator", input, 0, output, result);
    return result;
}

[[nodiscard]] bool evaluatePushAnimation(
    const arkheon::astsim::AnimationModelInput& input,
    arkheon::astsim::AnimationModelOutput& output) {
    const auto availableJointIds = collectAvailableJointIds(input);
    const bool result = evaluatePureKneeLiftAnimation(availableJointIds, input.simulationTimeSeconds, output);
    logEvaluationTrace("PushEvaluator", input, 1, output, result);
    return result;
}

[[nodiscard]] bool evaluateClimbAnimation(
    const arkheon::astsim::AnimationModelInput& input,
    arkheon::astsim::AnimationModelOutput& output) {
    const auto availableJointIds = collectAvailableJointIds(input);
    const bool result = evaluatePureKneeLiftAnimation(availableJointIds, input.simulationTimeSeconds, output);
    logEvaluationTrace("ClimbEvaluator", input, 2, output, result);
    return result;
}

[[nodiscard]] bool evaluateKneeLiftAnimation(
    const arkheon::astsim::AnimationModelInput& input,
    arkheon::astsim::AnimationModelOutput& output) {
    const auto availableJointIds = collectAvailableJointIds(input);
    const bool result = evaluatePureKneeLiftAnimation(availableJointIds, input.simulationTimeSeconds, output);
    logEvaluationTrace("KneeLiftEvaluator", input, 3, output, result);
    return result;
}

[[nodiscard]] bool evaluateIdleStoppedAnimation(
    const arkheon::astsim::AnimationModelInput& input,
    arkheon::astsim::AnimationModelOutput& output) {
    const auto availableJointIds = collectAvailableJointIds(input);
    const bool result = evaluatePureIdleStoppedAnimation(availableJointIds, output);
    logEvaluationTrace("IdleStoppedEvaluator", input, -1, output, result);
    return result;
}

} // namespace

int SimCharAnimNathanBalancedGaitPlugin::getInterfaceVersion() const {
    return 1;
}

arkheon::astlib::PluginMetadata SimCharAnimNathanBalancedGaitPlugin::getMetadata() const {
    arkheon::astlib::PluginMetadata metadata;
    metadata.setPluginId("sim-char-anim-nathan-balanced-gait");
    metadata.setVersion("1.0.0");
    metadata.setAuthor("Codex");
    return metadata;
}

void SimCharAnimNathanBalancedGaitPlugin::initialize(arkheon::astlib::PluginContext& context) {
    initialized_ = true;
    shutdown_ = false;
    registeredAnimationCodes_.clear();
    modelType_ = "animationModelNathanHuman";
    {
        std::lock_guard<std::mutex> guard(gEvaluationTraceMutex);
        gEvaluationTraceByKey.clear();
    }
    resetPluginLog();
    appendPluginLog("initialize modelType=" + modelType_);

    modelFactoryRegistry_ = nullptr;
    if (context.services) {
        auto* rawService = context.services->getService(arkheon::astsim::IModelPluginService::kPluginServiceId);
        auto* service = static_cast<arkheon::astsim::IModelPluginService*>(rawService);
        modelFactoryRegistry_ = service ? &service->modelFactoryRegistry() : nullptr;
    }

    if (!modelFactoryRegistry_) {
        return;
    }

    auto* prototypeBase = modelFactoryRegistry_->getRegisteredPrototype(modelType_);
    auto* prototypeAnimationModel = dynamic_cast<arkheon::astsim::IAnimationModel*>(prototypeBase);
    if (!prototypeAnimationModel) {
        return;
    }

    const std::vector<std::pair<std::string, arkheon::astsim::IAnimationModel::AnimationEvaluationFunction>> animations = {
        {"Idle Stopped", evaluateIdleStoppedAnimation},
        {"Knee Lift", evaluateKneeLiftAnimation},
        {"Walk", evaluateWalkAnimation},
        {"Push", evaluatePushAnimation},
        {"Climb", evaluateClimbAnimation}
    };

    for (const auto& animation : animations) {
        const bool registered = prototypeAnimationModel->registerAnimation(animation.first, animation.second);
        appendPluginLog("registerAnimation code=" + animation.first + " result=" + (registered ? std::string("true") : std::string("false")));
        if (registered) {
            registeredAnimationCodes_.push_back(animation.first);
        }
    }
}

void SimCharAnimNathanBalancedGaitPlugin::tick(double dt) {
    static_cast<void>(dt);
    if (!initialized_ || shutdown_ || !modelFactoryRegistry_) {
        return;
    }
}

void SimCharAnimNathanBalancedGaitPlugin::shutdown() {
    if (modelFactoryRegistry_ && !registeredAnimationCodes_.empty()) {
        auto* prototypeBase = modelFactoryRegistry_->getRegisteredPrototype(modelType_);
        auto* prototypeAnimationModel = dynamic_cast<arkheon::astsim::IAnimationModel*>(prototypeBase);
        if (prototypeAnimationModel) {
            for (const auto& animationCode : registeredAnimationCodes_) {
                static_cast<void>(prototypeAnimationModel->registerAnimation(
                    animationCode,
                    arkheon::astsim::IAnimationModel::AnimationEvaluationFunction {}));
            }
        }
    }

    registeredAnimationCodes_.clear();
    shutdown_ = true;
    modelFactoryRegistry_ = nullptr;
}

} // namespace arkheon::sample::simcharanimnathanbalancedgait

extern "C" {

ARKHEON_ASTLIB_API arkheon::astlib::IPlugin* create_plugin() {
    return new arkheon::sample::simcharanimnathanbalancedgait::SimCharAnimNathanBalancedGaitPlugin();
}

ARKHEON_ASTLIB_API void destroy_plugin(arkheon::astlib::IPlugin* plugin) {
    if (plugin) {
        delete plugin;
    }
}

ARKHEON_ASTLIB_API const char* get_plugin_signature() {
    return "ARKHEON_PLUGIN_V1";
}

} // extern "C"
