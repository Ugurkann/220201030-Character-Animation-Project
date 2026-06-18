#pragma once

#include <array>

namespace CharacterProject
{
constexpr int JointCount = 10;

enum JointIndex
{
    SpinePitch = 0,
    NeckYaw = 1,
    LeftShoulderPitch = 2,
    RightShoulderPitch = 3,
    LeftElbowPitch = 4,
    RightElbowPitch = 5,
    LeftHipPitch = 6,
    RightHipPitch = 7,
    LeftKneePitch = 8,
    RightKneePitch = 9
};

struct Vec3
{
    float x;
    float y;
    float z;

    Vec3();
    Vec3(float xValue, float yValue, float zValue);

    Vec3 operator+(const Vec3& other) const;
    Vec3 operator-(const Vec3& other) const;
    Vec3 operator*(float scalar) const;
    Vec3 operator/(float scalar) const;
    Vec3& operator+=(const Vec3& other);
    Vec3& operator-=(const Vec3& other);
    Vec3& operator*=(float scalar);

    float length() const;
    bool isFinite() const;

    static float dot(const Vec3& a, const Vec3& b);
    static Vec3 zero();
};

struct ContactState
{
    bool leftFootInContact;
    bool rightFootInContact;
    float leftFootWeight;
    float rightFootWeight;
    Vec3 leftFootPosition;
    Vec3 rightFootPosition;

    ContactState();
};

struct JointOutput
{
    std::array<float, JointCount> angles;

    JointOutput();

    float& operator[](int index);
    const float& operator[](int index) const;
};

struct HumanState
{
    float timeSeconds;
    float desiredSpeed;
    Vec3 rootPosition;
    Vec3 rootVelocity;
    Vec3 leftFootPosition;
    Vec3 rightFootPosition;
    ContactState contacts;
    std::array<float, JointCount> currentAngles;

    HumanState();
};

class JointController
{
public:
    JointController();

    void initialize();
    void reset();
    void update(float dt, const HumanState& state, JointOutput& output);

    Vec3 computeCenterOfMass(const HumanState& state) const;
    float clampAngle(int jointIndex, float angleRadians) const;
    bool validateAngle(int jointIndex, float angleRadians) const;

private:
    std::array<float, JointCount> previousAngles_;
    std::array<float, JointCount> minAngles_;
    std::array<float, JointCount> maxAngles_;
    float internalTime_;
    bool initialized_;

    void configureJointLimits();
    float sanitizeFloat(float value, float fallback) const;
    Vec3 sanitizeVec3(const Vec3& value, const Vec3& fallback) const;
    float clamp01(float value) const;
    float smoothStep(float edge0, float edge1, float value) const;
    float phase01(float radians) const;
    float currentAngleOrPrevious(const HumanState& state, int jointIndex) const;
    float smoothAngle(float current, float target, float dt) const;
    float normalizeContactWeight(bool inContact, float weight) const;
};
}
