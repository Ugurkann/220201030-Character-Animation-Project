#include "include/controller.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace CharacterProject
{
namespace
{
constexpr float Pi = 3.14159265358979323846f;
constexpr float TwoPi = Pi * 2.0f;
constexpr float DegreesToRadians = Pi / 180.0f;

float finiteOr(float value, float fallback)
{
    return std::isfinite(value) ? value : fallback;
}

float clampValue(float value, float minimum, float maximum)
{
    return std::max(minimum, std::min(value, maximum));
}

Vec3 lerpVec3(const Vec3& a, const Vec3& b, float t)
{
    return (a * (1.0f - t)) + (b * t);
}

float signedSin(float phase)
{
    return std::sin(phase);
}
}

Vec3::Vec3() : x(0.0f), y(0.0f), z(0.0f)
{
}

Vec3::Vec3(float xValue, float yValue, float zValue) : x(xValue), y(yValue), z(zValue)
{
}

Vec3 Vec3::operator+(const Vec3& other) const
{
    return Vec3(x + other.x, y + other.y, z + other.z);
}

Vec3 Vec3::operator-(const Vec3& other) const
{
    return Vec3(x - other.x, y - other.y, z - other.z);
}

Vec3 Vec3::operator*(float scalar) const
{
    return Vec3(x * scalar, y * scalar, z * scalar);
}

Vec3 Vec3::operator/(float scalar) const
{
    if (!std::isfinite(scalar) || std::fabs(scalar) <= std::numeric_limits<float>::epsilon())
    {
        return Vec3::zero();
    }

    return Vec3(x / scalar, y / scalar, z / scalar);
}

Vec3& Vec3::operator+=(const Vec3& other)
{
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
}

Vec3& Vec3::operator-=(const Vec3& other)
{
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
}

Vec3& Vec3::operator*=(float scalar)
{
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
}

float Vec3::length() const
{
    return std::sqrt((x * x) + (y * y) + (z * z));
}

bool Vec3::isFinite() const
{
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

float Vec3::dot(const Vec3& a, const Vec3& b)
{
    return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

Vec3 Vec3::zero()
{
    return Vec3(0.0f, 0.0f, 0.0f);
}

ContactState::ContactState()
    : leftFootInContact(true),
      rightFootInContact(true),
      leftFootWeight(0.5f),
      rightFootWeight(0.5f),
      leftFootPosition(-0.10f, 0.0f, 0.0f),
      rightFootPosition(0.10f, 0.0f, 0.0f)
{
}

JointOutput::JointOutput()
{
    angles.fill(0.0f);
}

float& JointOutput::operator[](int index)
{
    return angles[static_cast<std::size_t>(index)];
}

const float& JointOutput::operator[](int index) const
{
    return angles[static_cast<std::size_t>(index)];
}

HumanState::HumanState()
    : timeSeconds(0.0f),
      desiredSpeed(0.0f),
      rootPosition(0.0f, 1.0f, 0.0f),
      rootVelocity(0.0f, 0.0f, 0.0f),
      leftFootPosition(-0.10f, 0.0f, 0.0f),
      rightFootPosition(0.10f, 0.0f, 0.0f),
      contacts()
{
    currentAngles.fill(0.0f);
}

JointController::JointController()
    : previousAngles_(),
      minAngles_(),
      maxAngles_(),
      internalTime_(0.0f),
      initialized_(false)
{
    configureJointLimits();
    reset();
}

void JointController::initialize()
{
    configureJointLimits();
    reset();
    initialized_ = true;
}

void JointController::reset()
{
    previousAngles_.fill(0.0f);
    internalTime_ = 0.0f;
}

void JointController::update(float dt, const HumanState& state, JointOutput& output)
{
    if (!initialized_)
    {
        initialize();
    }

    const float safeDt = clampValue(sanitizeFloat(dt, 0.0f), 0.0f, 0.1f);
    internalTime_ += safeDt;
    if (internalTime_ > 10000.0f)
    {
        internalTime_ = std::fmod(internalTime_, 10000.0f);
    }

    const float externalTime = sanitizeFloat(state.timeSeconds, -1.0f);
    const float time = externalTime > 0.0f ? externalTime : internalTime_;
    const Vec3 rootVelocity = sanitizeVec3(state.rootVelocity, Vec3::zero());
    const float speedInput = std::fabs(state.desiredSpeed) > 0.001f ? state.desiredSpeed : rootVelocity.z;
    const float desiredSpeed = clampValue(sanitizeFloat(speedInput, 0.0f), -2.5f, 2.5f);
    const float speedMagnitude = clampValue(std::fabs(desiredSpeed), 0.0f, 2.5f);
    const float locomotionBlend = smoothStep(0.05f, 1.25f, speedMagnitude);
    const float direction = desiredSpeed < 0.0f ? -1.0f : 1.0f;

    const float gaitFrequency = 1.15f + (speedMagnitude * 0.42f);
    const float gaitPhase = std::fmod(time * TwoPi * gaitFrequency, TwoPi);
    const float leftPhase01 = phase01(gaitPhase);
    const float rightPhase01 = phase01(gaitPhase + Pi);
    const float leftWave = signedSin(gaitPhase);
    const float rightWave = signedSin(gaitPhase + Pi);

    const float proceduralLeftContact = 1.0f - smoothStep(0.38f, 0.56f, leftPhase01);
    const float proceduralRightContact = 1.0f - smoothStep(0.38f, 0.56f, rightPhase01);
    const float measuredLeftContact = normalizeContactWeight(state.contacts.leftFootInContact, state.contacts.leftFootWeight);
    const float measuredRightContact = normalizeContactWeight(state.contacts.rightFootInContact, state.contacts.rightFootWeight);
    const bool hasMeasuredSupport = (measuredLeftContact + measuredRightContact) > 0.01f;
    const float leftContact = hasMeasuredSupport ? measuredLeftContact : proceduralLeftContact;
    const float rightContact = hasMeasuredSupport ? measuredRightContact : proceduralRightContact;
    const float leftSwing = clamp01(1.0f - leftContact) * smoothStep(0.45f, 0.70f, leftPhase01);
    const float rightSwing = clamp01(1.0f - rightContact) * smoothStep(0.45f, 0.70f, rightPhase01);

    const Vec3 centerOfMass = computeCenterOfMass(state);
    const Vec3 leftFoot = sanitizeVec3(state.leftFootPosition, sanitizeVec3(state.contacts.leftFootPosition, Vec3(-0.10f, 0.0f, 0.0f)));
    const Vec3 rightFoot = sanitizeVec3(state.rightFootPosition, sanitizeVec3(state.contacts.rightFootPosition, Vec3(0.10f, 0.0f, 0.0f)));
    const float totalContact = std::max(0.001f, leftContact + rightContact);
    const Vec3 weightedSupport = ((leftFoot * leftContact) + (rightFoot * rightContact)) / totalContact;
    const Vec3 supportCenter = totalContact > 0.01f ? weightedSupport : lerpVec3(leftFoot, rightFoot, 0.5f);
    const float supportWidth = clampValue(std::fabs(leftFoot.x - rightFoot.x), 0.12f, 0.45f);
    const float sagittalRange = 0.11f + locomotionBlend * 0.07f;
    const float lateralRange = supportWidth * (totalContact > 1.2f ? 0.45f : 0.28f);
    const float sagittalError = clampValue((centerOfMass.z - supportCenter.z) / sagittalRange, -1.0f, 1.0f);
    const float lateralError = clampValue((centerOfMass.x - supportCenter.x) / std::max(0.04f, lateralRange), -1.0f, 1.0f);
    const float balancePitchCorrection = clampValue(-sagittalError * 6.5f * DegreesToRadians, -12.0f * DegreesToRadians, 12.0f * DegreesToRadians);
    const float balanceYawCorrection = clampValue(-lateralError * 5.0f * DegreesToRadians, -8.0f * DegreesToRadians, 8.0f * DegreesToRadians);
    const float stanceAsymmetry = clampValue(rightContact - leftContact, -1.0f, 1.0f);

    std::array<float, JointCount> target{};
    target.fill(0.0f);

    const float idlePhase = time * TwoPi * 0.32f;
    const float idleBreath = std::sin(idlePhase) * 1.6f * DegreesToRadians;
    const float idleHeadScan = std::sin(idlePhase * 0.45f) * 2.0f * DegreesToRadians;
    const float verticalPulse = (1.0f - std::cos(gaitPhase * 2.0f)) * 0.5f;
    const float weightShift = stanceAsymmetry * locomotionBlend * 3.0f * DegreesToRadians;
    const float armAmplitude = locomotionBlend * (20.0f + speedMagnitude * 5.0f) * DegreesToRadians;
    const float hipAmplitude = locomotionBlend * (20.0f + speedMagnitude * 4.5f) * DegreesToRadians;
    const float kneeAmplitude = locomotionBlend * (30.0f + speedMagnitude * 9.0f) * DegreesToRadians;
    const float elbowBase = (7.0f + locomotionBlend * 5.0f) * DegreesToRadians;

    target[SpinePitch] = idleBreath + (direction * locomotionBlend * 3.5f * DegreesToRadians) + balancePitchCorrection + (verticalPulse * locomotionBlend * 1.3f * DegreesToRadians);
    target[NeckYaw] = idleHeadScan * (1.0f - locomotionBlend * 0.55f) + balanceYawCorrection;

    target[LeftShoulderPitch] = (-leftWave * armAmplitude * direction) + (idleBreath * 0.35f) + (weightShift * 0.25f);
    target[RightShoulderPitch] = (-rightWave * armAmplitude * direction) - (idleBreath * 0.35f) - (weightShift * 0.25f);
    target[LeftElbowPitch] = elbowBase + (std::max(0.0f, -leftWave) * locomotionBlend * 13.0f * DegreesToRadians);
    target[RightElbowPitch] = elbowBase + (std::max(0.0f, -rightWave) * locomotionBlend * 13.0f * DegreesToRadians);

    const float leftHeelStrikeDamping = leftContact * smoothStep(0.00f, 0.18f, leftPhase01);
    const float rightHeelStrikeDamping = rightContact * smoothStep(0.00f, 0.18f, rightPhase01);
    const float leftHipSwing = leftWave * hipAmplitude * direction * (0.50f + 0.50f * leftSwing);
    const float rightHipSwing = rightWave * hipAmplitude * direction * (0.50f + 0.50f * rightSwing);
    target[LeftHipPitch] = leftHipSwing + (balancePitchCorrection * 0.42f) - (leftHeelStrikeDamping * 2.5f * DegreesToRadians);
    target[RightHipPitch] = rightHipSwing + (balancePitchCorrection * 0.42f) - (rightHeelStrikeDamping * 2.5f * DegreesToRadians);

    const float leftKneeLift = leftSwing * kneeAmplitude;
    const float rightKneeLift = rightSwing * kneeAmplitude;
    const float leftStanceFlex = leftContact * locomotionBlend * (2.0f + verticalPulse * 3.0f) * DegreesToRadians;
    const float rightStanceFlex = rightContact * locomotionBlend * (2.0f + verticalPulse * 3.0f) * DegreesToRadians;
    target[LeftKneePitch] = leftKneeLift + leftStanceFlex;
    target[RightKneePitch] = rightKneeLift + rightStanceFlex;

    if (locomotionBlend < 0.05f)
    {
        target[LeftShoulderPitch] = idleBreath * 0.35f;
        target[RightShoulderPitch] = -idleBreath * 0.35f;
        target[LeftElbowPitch] = 6.0f * DegreesToRadians;
        target[RightElbowPitch] = 6.0f * DegreesToRadians;
        target[LeftHipPitch] = std::sin(idlePhase + Pi) * 0.8f * DegreesToRadians + balancePitchCorrection * 0.25f;
        target[RightHipPitch] = std::sin(idlePhase) * 0.8f * DegreesToRadians + balancePitchCorrection * 0.25f;
        target[LeftKneePitch] = 1.2f * DegreesToRadians;
        target[RightKneePitch] = 1.2f * DegreesToRadians;
    }

    for (int i = 0; i < JointCount; ++i)
    {
        const std::size_t index = static_cast<std::size_t>(i);
        const float safeTarget = sanitizeFloat(target[index], previousAngles_[index]);
        const float clampedTarget = clampAngle(i, safeTarget);
        const float smoothed = smoothAngle(previousAngles_[index], clampedTarget, safeDt);
        const float finalAngle = clampAngle(i, sanitizeFloat(smoothed, 0.0f));

        output.angles[index] = finalAngle;
        previousAngles_[index] = finalAngle;
    }
}

Vec3 JointController::computeCenterOfMass(const HumanState& state) const
{
    const Vec3 root = sanitizeVec3(state.rootPosition, Vec3(0.0f, 1.0f, 0.0f));
    const Vec3 velocity = sanitizeVec3(state.rootVelocity, Vec3::zero());
    const Vec3 leftFoot = sanitizeVec3(state.leftFootPosition, sanitizeVec3(state.contacts.leftFootPosition, Vec3(-0.10f, 0.0f, 0.0f)));
    const Vec3 rightFoot = sanitizeVec3(state.rightFootPosition, sanitizeVec3(state.contacts.rightFootPosition, Vec3(0.10f, 0.0f, 0.0f)));

    const float leftHip = currentAngleOrPrevious(state, LeftHipPitch);
    const float rightHip = currentAngleOrPrevious(state, RightHipPitch);
    const float leftKnee = currentAngleOrPrevious(state, LeftKneePitch);
    const float rightKnee = currentAngleOrPrevious(state, RightKneePitch);
    const float spine = currentAngleOrPrevious(state, SpinePitch);
    const float leftShoulder = currentAngleOrPrevious(state, LeftShoulderPitch);
    const float rightShoulder = currentAngleOrPrevious(state, RightShoulderPitch);
    const float leftElbow = currentAngleOrPrevious(state, LeftElbowPitch);
    const float rightElbow = currentAngleOrPrevious(state, RightElbowPitch);

    const Vec3 pelvis = root + Vec3(0.0f, -0.10f, 0.0f);
    const Vec3 torso = root + Vec3(0.0f, 0.28f, std::sin(spine) * 0.18f);
    const Vec3 head = root + Vec3(0.0f, 0.58f, std::sin(spine) * 0.24f);

    const Vec3 leftHipPos = root + Vec3(-0.09f, -0.10f, 0.0f);
    const Vec3 rightHipPos = root + Vec3(0.09f, -0.10f, 0.0f);
    const Vec3 leftKneePos = leftHipPos + Vec3(0.0f, -0.43f * std::cos(leftHip), 0.43f * std::sin(leftHip));
    const Vec3 rightKneePos = rightHipPos + Vec3(0.0f, -0.43f * std::cos(rightHip), 0.43f * std::sin(rightHip));
    const Vec3 leftAnkleEstimate = leftKneePos + Vec3(0.0f, -0.42f * std::cos(leftHip - leftKnee), 0.42f * std::sin(leftHip - leftKnee));
    const Vec3 rightAnkleEstimate = rightKneePos + Vec3(0.0f, -0.42f * std::cos(rightHip - rightKnee), 0.42f * std::sin(rightHip - rightKnee));
    const Vec3 leftAnkle = lerpVec3(leftAnkleEstimate, leftFoot, 0.45f);
    const Vec3 rightAnkle = lerpVec3(rightAnkleEstimate, rightFoot, 0.45f);

    const Vec3 leftShoulderPos = root + Vec3(-0.18f, 0.30f, 0.0f);
    const Vec3 rightShoulderPos = root + Vec3(0.18f, 0.30f, 0.0f);
    const Vec3 leftElbowPos = leftShoulderPos + Vec3(0.0f, -0.25f * std::cos(leftShoulder), 0.25f * std::sin(leftShoulder));
    const Vec3 rightElbowPos = rightShoulderPos + Vec3(0.0f, -0.25f * std::cos(rightShoulder), 0.25f * std::sin(rightShoulder));
    const Vec3 leftHandPos = leftElbowPos + Vec3(0.0f, -0.23f * std::cos(leftShoulder - leftElbow), 0.23f * std::sin(leftShoulder - leftElbow));
    const Vec3 rightHandPos = rightElbowPos + Vec3(0.0f, -0.23f * std::cos(rightShoulder - rightElbow), 0.23f * std::sin(rightShoulder - rightElbow));

    Vec3 weighted = Vec3::zero();
    float mass = 0.0f;
    const auto addSegment = [&weighted, &mass](const Vec3& point, float segmentMass) {
        weighted += point * segmentMass;
        mass += segmentMass;
    };

    addSegment(pelvis, 0.18f);
    addSegment(torso, 0.34f);
    addSegment(head, 0.08f);
    addSegment(lerpVec3(leftHipPos, leftKneePos, 0.43f), 0.10f);
    addSegment(lerpVec3(rightHipPos, rightKneePos, 0.43f), 0.10f);
    addSegment(lerpVec3(leftKneePos, leftAnkle, 0.43f), 0.045f);
    addSegment(lerpVec3(rightKneePos, rightAnkle, 0.43f), 0.045f);
    addSegment(lerpVec3(leftShoulderPos, leftElbowPos, 0.46f), 0.035f);
    addSegment(lerpVec3(rightShoulderPos, rightElbowPos, 0.46f), 0.035f);
    addSegment(lerpVec3(leftElbowPos, leftHandPos, 0.45f), 0.02f);
    addSegment(lerpVec3(rightElbowPos, rightHandPos, 0.45f), 0.02f);

    Vec3 center = weighted / std::max(0.001f, mass);
    center.z += clampValue(velocity.z, -2.5f, 2.5f) * 0.035f;
    center.x += clampValue(velocity.x, -1.5f, 1.5f) * 0.025f;
    return center;
}

float JointController::clampAngle(int jointIndex, float angleRadians) const
{
    if (jointIndex < 0 || jointIndex >= JointCount)
    {
        return 0.0f;
    }

    const std::size_t index = static_cast<std::size_t>(jointIndex);
    const float safeAngle = sanitizeFloat(angleRadians, 0.0f);
    return clampValue(safeAngle, minAngles_[index], maxAngles_[index]);
}

bool JointController::validateAngle(int jointIndex, float angleRadians) const
{
    if (jointIndex < 0 || jointIndex >= JointCount || !std::isfinite(angleRadians))
    {
        return false;
    }

    const std::size_t index = static_cast<std::size_t>(jointIndex);
    return angleRadians >= minAngles_[index] && angleRadians <= maxAngles_[index];
}

void JointController::configureJointLimits()
{
    minAngles_[SpinePitch] = -25.0f * DegreesToRadians;
    maxAngles_[SpinePitch] = 25.0f * DegreesToRadians;
    minAngles_[NeckYaw] = -55.0f * DegreesToRadians;
    maxAngles_[NeckYaw] = 55.0f * DegreesToRadians;
    minAngles_[LeftShoulderPitch] = -75.0f * DegreesToRadians;
    maxAngles_[LeftShoulderPitch] = 95.0f * DegreesToRadians;
    minAngles_[RightShoulderPitch] = -75.0f * DegreesToRadians;
    maxAngles_[RightShoulderPitch] = 95.0f * DegreesToRadians;
    minAngles_[LeftElbowPitch] = 0.0f * DegreesToRadians;
    maxAngles_[LeftElbowPitch] = 145.0f * DegreesToRadians;
    minAngles_[RightElbowPitch] = 0.0f * DegreesToRadians;
    maxAngles_[RightElbowPitch] = 145.0f * DegreesToRadians;
    minAngles_[LeftHipPitch] = -45.0f * DegreesToRadians;
    maxAngles_[LeftHipPitch] = 80.0f * DegreesToRadians;
    minAngles_[RightHipPitch] = -45.0f * DegreesToRadians;
    maxAngles_[RightHipPitch] = 80.0f * DegreesToRadians;
    minAngles_[LeftKneePitch] = 0.0f * DegreesToRadians;
    maxAngles_[LeftKneePitch] = 130.0f * DegreesToRadians;
    minAngles_[RightKneePitch] = 0.0f * DegreesToRadians;
    maxAngles_[RightKneePitch] = 130.0f * DegreesToRadians;
}

float JointController::sanitizeFloat(float value, float fallback) const
{
    return finiteOr(value, fallback);
}

Vec3 JointController::sanitizeVec3(const Vec3& value, const Vec3& fallback) const
{
    return value.isFinite() ? value : fallback;
}

float JointController::clamp01(float value) const
{
    return clampValue(sanitizeFloat(value, 0.0f), 0.0f, 1.0f);
}

float JointController::smoothStep(float edge0, float edge1, float value) const
{
    if (edge0 == edge1)
    {
        return value < edge0 ? 0.0f : 1.0f;
    }

    const float t = clamp01((value - edge0) / (edge1 - edge0));
    return t * t * (3.0f - (2.0f * t));
}

float JointController::phase01(float radians) const
{
    float wrapped = std::fmod(radians, TwoPi);
    if (wrapped < 0.0f)
    {
        wrapped += TwoPi;
    }

    return wrapped / TwoPi;
}

float JointController::currentAngleOrPrevious(const HumanState& state, int jointIndex) const
{
    if (jointIndex < 0 || jointIndex >= JointCount)
    {
        return 0.0f;
    }

    const std::size_t index = static_cast<std::size_t>(jointIndex);
    return sanitizeFloat(state.currentAngles[index], previousAngles_[index]);
}

float JointController::smoothAngle(float current, float target, float dt) const
{
    if (dt <= 0.0f)
    {
        return target;
    }

    const float response = 13.5f;
    const float alpha = 1.0f - std::exp(-response * dt);
    return current + ((target - current) * clamp01(alpha));
}

float JointController::normalizeContactWeight(bool inContact, float weight) const
{
    if (!inContact)
    {
        return 0.0f;
    }

    return clamp01(weight);
}
}
