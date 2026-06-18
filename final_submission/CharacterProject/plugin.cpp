#include "include/arkheon/character/ICharacterController.h"
#include "include/controller.h"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <new>

namespace
{
constexpr float Pi = 3.14159265358979323846f;
constexpr float TwoPi = Pi * 2.0f;
constexpr float FixedDt = 0.02f;
constexpr int HidA = 4;
constexpr int HidD = 7;
constexpr int HidS = 22;
constexpr int HidW = 26;
constexpr int HidLeftShift = 225;
constexpr const char* DebugLogPath = "C:\\N8RO\\userPlugins\\sim\\character_plugin_debug.log";

struct N8roGenericPluginHandle
{
    uint32_t marker;
};

struct Controller
{
    CharacterProject::JointController jointController;
    CharacterProject::HumanState humanState;
    CharacterProject::JointOutput jointOutput;
    float segmentLengths[ARK_JOINT_COUNT];
    float rootYawRad;
    int32_t activeMotion;
    int32_t lastGoalSequence;
    int32_t currentPathLength;
    int32_t currentPathIndex;
    arkheon_vec3 path[32];
    arkheon_bone_override lastOverrides[ARK_JOINT_COUNT];
    arkheon_vec3 angularVelocity[ARK_JOINT_COUNT];
    arkheon_quat simulatedRotation[ARK_JOINT_COUNT];
    arkheon_vec3 lastRootTranslation;
    arkheon_quat lastRootRotation;
    bool physicsInitialized;

    Controller()
        : segmentLengths{},
          rootYawRad(0.0f),
          activeMotion(0),
          lastGoalSequence(-1),
          currentPathLength(0),
          currentPathIndex(0),
          path{},
          lastOverrides{},
          angularVelocity{},
          simulatedRotation{},
          lastRootTranslation{0.0f, 0.0f, 0.0f},
          lastRootRotation{0.0f, 0.0f, 0.0f, 1.0f},
          physicsInitialized(false)
    {
        jointController.initialize();
        for (int i = 0; i < ARK_JOINT_COUNT; ++i)
        {
            lastOverrides[i].local_rotation = {0.0f, 0.0f, 0.0f, 1.0f};
            lastOverrides[i].apply = 1;
            angularVelocity[i] = {0.0f, 0.0f, 0.0f};
            simulatedRotation[i] = {0.0f, 0.0f, 0.0f, 1.0f};
        }
    }
};

void ensureDebugLogDirectory()
{
#ifdef _WIN32
    CreateDirectoryA("C:\\N8RO", nullptr);
    CreateDirectoryA("C:\\N8RO\\userPlugins", nullptr);
    CreateDirectoryA("C:\\N8RO\\userPlugins\\sim", nullptr);
#endif
}

void appendDebugLog(const char* text)
{
#ifdef _WIN32
    if (text == nullptr)
    {
        return;
    }

    ensureDebugLogDirectory();
    HANDLE file = CreateFileA(
        DebugLogPath,
        FILE_APPEND_DATA,
        FILE_SHARE_READ | FILE_SHARE_WRITE,
        nullptr,
        OPEN_ALWAYS,
        FILE_ATTRIBUTE_NORMAL,
        nullptr);

    if (file == INVALID_HANDLE_VALUE)
    {
        return;
    }

    DWORD written = 0;
    const DWORD length = static_cast<DWORD>(std::strlen(text));
    WriteFile(file, text, length, &written, nullptr);
    CloseHandle(file);
#else
    (void)text;
#endif
}

void traceExport(const char* functionName, const char* eventName)
{
    char message[512] = {};
#ifdef _WIN32
    std::snprintf(
        message,
        sizeof(message),
        "[character_plugin_ugurkan] pid=%lu tid=%lu %s %s\n",
        static_cast<unsigned long>(GetCurrentProcessId()),
        static_cast<unsigned long>(GetCurrentThreadId()),
        functionName != nullptr ? functionName : "<unknown>",
        eventName != nullptr ? eventName : "<event>");
    OutputDebugStringA(message);
    appendDebugLog(message);
#else
    std::snprintf(
        message,
        sizeof(message),
        "[character_plugin_ugurkan] %s %s\n",
        functionName != nullptr ? functionName : "<unknown>",
        eventName != nullptr ? eventName : "<event>");
    appendDebugLog(message);
#endif
}

void traceExportStatus(const char* functionName, const char* eventName, int32_t status)
{
    char message[512] = {};
#ifdef _WIN32
    std::snprintf(
        message,
        sizeof(message),
        "[character_plugin_ugurkan] pid=%lu tid=%lu %s %s status=%d\n",
        static_cast<unsigned long>(GetCurrentProcessId()),
        static_cast<unsigned long>(GetCurrentThreadId()),
        functionName != nullptr ? functionName : "<unknown>",
        eventName != nullptr ? eventName : "<event>",
        static_cast<int>(status));
    OutputDebugStringA(message);
    appendDebugLog(message);
#else
    std::snprintf(
        message,
        sizeof(message),
        "[character_plugin_ugurkan] %s %s status=%d\n",
        functionName != nullptr ? functionName : "<unknown>",
        eventName != nullptr ? eventName : "<event>",
        static_cast<int>(status));
    appendDebugLog(message);
#endif
}

float finiteOr(float value, float fallback)
{
    return std::isfinite(value) ? value : fallback;
}

float clampValue(float value, float minimum, float maximum)
{
    return std::max(minimum, std::min(value, maximum));
}

float length2D(const arkheon_vec3& value)
{
    return std::sqrt((value.x * value.x) + (value.z * value.z));
}

arkheon_vec3 subtract(const arkheon_vec3& a, const arkheon_vec3& b)
{
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

arkheon_vec3 add(const arkheon_vec3& a, const arkheon_vec3& b)
{
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

arkheon_vec3 multiplyVec(const arkheon_vec3& value, float scalar)
{
    return {value.x * scalar, value.y * scalar, value.z * scalar};
}

float dot(const arkheon_vec3& a, const arkheon_vec3& b)
{
    return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

float length3D(const arkheon_vec3& value)
{
    return std::sqrt(dot(value, value));
}

arkheon_vec3 normalize2D(const arkheon_vec3& value)
{
    const float length = length2D(value);
    if (length <= 0.0001f || !std::isfinite(length))
    {
        return {0.0f, 0.0f, 0.0f};
    }

    return {value.x / length, 0.0f, value.z / length};
}

arkheon_quat normalizeQuat(arkheon_quat q)
{
    const float n2 = (q.x * q.x) + (q.y * q.y) + (q.z * q.z) + (q.w * q.w);
    if (!std::isfinite(n2) || n2 <= 0.000001f)
    {
        return {0.0f, 0.0f, 0.0f, 1.0f};
    }

    const float invLength = 1.0f / std::sqrt(n2);
    return {q.x * invLength, q.y * invLength, q.z * invLength, q.w * invLength};
}

arkheon_quat axisAngle(float x, float y, float z, float angle)
{
    const float half = angle * 0.5f;
    const float s = std::sin(half);
    return normalizeQuat({x * s, y * s, z * s, std::cos(half)});
}

arkheon_quat multiply(arkheon_quat a, arkheon_quat b)
{
    return normalizeQuat({
        (a.w * b.x) + (a.x * b.w) + (a.y * b.z) - (a.z * b.y),
        (a.w * b.y) - (a.x * b.z) + (a.y * b.w) + (a.z * b.x),
        (a.w * b.z) + (a.x * b.y) - (a.y * b.x) + (a.z * b.w),
        (a.w * b.w) - (a.x * b.x) - (a.y * b.y) - (a.z * b.z)
    });
}

arkheon_quat conjugate(arkheon_quat q)
{
    return {-q.x, -q.y, -q.z, q.w};
}

arkheon_quat negateQuat(arkheon_quat q)
{
    return {-q.x, -q.y, -q.z, -q.w};
}

arkheon_vec3 shortestArcError(arkheon_quat target, arkheon_quat current)
{
    arkheon_quat error = multiply(normalizeQuat(target), conjugate(normalizeQuat(current)));
    if (error.w < 0.0f)
    {
        error = negateQuat(error);
    }

    const float vectorLength = std::sqrt((error.x * error.x) + (error.y * error.y) + (error.z * error.z));
    if (!std::isfinite(vectorLength) || vectorLength < 0.000001f)
    {
        return {0.0f, 0.0f, 0.0f};
    }

    const float angle = 2.0f * std::atan2(vectorLength, clampValue(error.w, -1.0f, 1.0f));
    const float scale = angle / vectorLength;
    return {error.x * scale, error.y * scale, error.z * scale};
}

arkheon_quat integrateOrientation(arkheon_quat current, const arkheon_vec3& angularVelocity, float dt)
{
    const float speed = length3D(angularVelocity);
    if (!std::isfinite(speed) || speed < 0.000001f)
    {
        return normalizeQuat(current);
    }

    const float invSpeed = 1.0f / speed;
    const arkheon_quat delta = axisAngle(
        angularVelocity.x * invSpeed,
        angularVelocity.y * invSpeed,
        angularVelocity.z * invSpeed,
        speed * dt);
    return normalizeQuat(multiply(delta, current));
}

arkheon_quat yawQuat(float yaw)
{
    return axisAngle(0.0f, 1.0f, 0.0f, yaw);
}

float radiansFromDegrees(float degrees)
{
    return degrees * (Pi / 180.0f);
}

float blendAngle(float base, float overlay, float weight)
{
    const float safeWeight = clampValue(finiteOr(weight, 0.0f), 0.0f, 1.0f);
    return base + ((overlay - base) * safeWeight);
}

struct JointGains
{
    float kp;
    float kd;
};

JointGains gainsForJoint(int joint)
{
    switch (joint)
    {
    case ARK_JOINT_UPPERARM_L:
    case ARK_JOINT_UPPERARM_R:
        return {46.0f, 8.5f};
    case ARK_JOINT_LOWERARM_L:
    case ARK_JOINT_LOWERARM_R:
        return {58.0f, 9.0f};
    case ARK_JOINT_THIGH_L:
    case ARK_JOINT_THIGH_R:
        return {70.0f, 11.5f};
    case ARK_JOINT_CALF_L:
    case ARK_JOINT_CALF_R:
        return {64.0f, 10.0f};
    case ARK_JOINT_FOOT_L:
    case ARK_JOINT_FOOT_R:
        return {38.0f, 7.0f};
    default:
        return {45.0f, 8.0f};
    }
}

float inertiaForJoint(const Controller& controller, int joint)
{
    float length = controller.segmentLengths[joint];
    if (!(std::isfinite(length) && length > 0.001f))
    {
        switch (joint)
        {
        case ARK_JOINT_UPPERARM_L:
        case ARK_JOINT_UPPERARM_R:
            length = controller.segmentLengths[ARK_JOINT_LOWERARM_L] > 0.001f ? controller.segmentLengths[ARK_JOINT_LOWERARM_L] : 0.30f;
            break;
        case ARK_JOINT_THIGH_L:
        case ARK_JOINT_THIGH_R:
            length = controller.segmentLengths[ARK_JOINT_CALF_L] > 0.001f ? controller.segmentLengths[ARK_JOINT_CALF_L] : 0.41f;
            break;
        default:
            length = 0.25f;
            break;
        }
    }

    const float limbScale = length * length;
    switch (joint)
    {
    case ARK_JOINT_THIGH_L:
    case ARK_JOINT_THIGH_R:
        return 0.08f + limbScale * 0.55f;
    case ARK_JOINT_CALF_L:
    case ARK_JOINT_CALF_R:
        return 0.05f + limbScale * 0.38f;
    case ARK_JOINT_UPPERARM_L:
    case ARK_JOINT_UPPERARM_R:
        return 0.035f + limbScale * 0.28f;
    case ARK_JOINT_LOWERARM_L:
    case ARK_JOINT_LOWERARM_R:
        return 0.025f + limbScale * 0.20f;
    case ARK_JOINT_FOOT_L:
    case ARK_JOINT_FOOT_R:
        return 0.02f + limbScale * 0.16f;
    default:
        return 0.06f;
    }
}

CharacterProject::Vec3 toVec3(const arkheon_vec3& value)
{
    return CharacterProject::Vec3(
        finiteOr(value.x, 0.0f),
        finiteOr(value.y, 0.0f),
        finiteOr(value.z, 0.0f));
}

arkheon_vec3 rootPositionFromBones(const arkheon_bone_state inBones[66])
{
    if (inBones != nullptr)
    {
        const arkheon_vec3 pelvis = inBones[0].world_position;
        if (std::isfinite(pelvis.x) && std::isfinite(pelvis.y) && std::isfinite(pelvis.z))
        {
            return pelvis;
        }
    }

    return {0.0f, 0.0f, 0.0f};
}

bool keyHeld(const arkheon_input_state* input, int key)
{
    return input != nullptr && key >= 0 && key < 256 && input->keys[key] != 0;
}

void updateMotionHotkeys(Controller& controller, const arkheon_input_state* input)
{
    if (input == nullptr)
    {
        return;
    }

    if (input->hotkey_motion_a != 0)
    {
        controller.activeMotion = 0;
    }
    if (input->hotkey_motion_b != 0)
    {
        controller.activeMotion = 1;
    }
    if (input->hotkey_motion_c != 0)
    {
        controller.activeMotion = 2;
    }
}

arkheon_vec3 computeManualLocomotion(const arkheon_input_state* input, float dt, float& outYaw)
{
    if (input == nullptr)
    {
        return {0.0f, 0.0f, 0.0f};
    }

    outYaw = finiteOr(input->look_yaw_rad, outYaw);
    const float forwardIntent = (keyHeld(input, HidW) ? 1.0f : 0.0f) - (keyHeld(input, HidS) ? 1.0f : 0.0f);
    const float strafeIntent = (keyHeld(input, HidD) ? 1.0f : 0.0f) - (keyHeld(input, HidA) ? 1.0f : 0.0f);
    const float intentLength = std::sqrt((forwardIntent * forwardIntent) + (strafeIntent * strafeIntent));
    if (intentLength <= 0.0001f)
    {
        return {0.0f, 0.0f, 0.0f};
    }

    const float invIntentLength = 1.0f / intentLength;
    const float forward = forwardIntent * invIntentLength;
    const float strafe = strafeIntent * invIntentLength;
    const float sinYaw = std::sin(outYaw);
    const float cosYaw = std::cos(outYaw);
    const float speed = keyHeld(input, HidLeftShift) ? 4.0f : 1.4f;

    return {
        ((sinYaw * forward) + (cosYaw * strafe)) * speed * dt,
        0.0f,
        ((cosYaw * forward) - (sinYaw * strafe)) * speed * dt
    };
}

void resetPath(Controller& controller)
{
    controller.currentPathLength = 0;
    controller.currentPathIndex = 0;
    for (arkheon_vec3& point : controller.path)
    {
        point = {0.0f, 0.0f, 0.0f};
    }
}

arkheon_vec3 computeGoalLocomotion(Controller& controller,
                                   const arkheon_mission_goal* goal,
                                   const arkheon_env_api* env,
                                   arkheon_vec3 currentRoot,
                                   float dt)
{
    if (goal == nullptr || goal->type == ARK_GOAL_NONE)
    {
        return {0.0f, 0.0f, 0.0f};
    }

    if (goal->sequence_id != controller.lastGoalSequence)
    {
        controller.lastGoalSequence = goal->sequence_id;
        resetPath(controller);
        if (env != nullptr && env->navmesh_query != nullptr)
        {
            const int32_t pathLength = env->navmesh_query(
                env->host_ctx,
                currentRoot,
                goal->target_position,
                controller.path,
                32);
            controller.currentPathLength = clampValue(static_cast<float>(pathLength), 0.0f, 32.0f) > 0.0f ? pathLength : 0;
        }
        if (controller.currentPathLength <= 0)
        {
            controller.path[0] = goal->target_position;
            controller.currentPathLength = 1;
        }
    }

    if (goal->type == ARK_GOAL_GOTO)
    {
        const float tolerance = goal->tolerance_m > 0.0f ? goal->tolerance_m : 0.3f;
        const arkheon_vec3 toGoal = subtract(goal->target_position, currentRoot);
        if (length2D(toGoal) <= tolerance)
        {
            if (env != nullptr && env->report_goal_complete != nullptr)
            {
                env->report_goal_complete(env->host_ctx, goal->sequence_id, ARK_GOAL_RESULT_OK);
            }
            return {0.0f, 0.0f, 0.0f};
        }

        while (controller.currentPathIndex < controller.currentPathLength - 1)
        {
            const arkheon_vec3 toPoint = subtract(controller.path[controller.currentPathIndex], currentRoot);
            if (length2D(toPoint) > 0.25f)
            {
                break;
            }
            ++controller.currentPathIndex;
        }

        const arkheon_vec3 target = controller.path[controller.currentPathIndex];
        const arkheon_vec3 direction = normalize2D(subtract(target, currentRoot));
        return {direction.x * 1.25f * dt, 0.0f, direction.z * 1.25f * dt};
    }

    if ((goal->type == ARK_GOAL_PUSH || goal->type == ARK_GOAL_INTERACT) &&
        env != nullptr && env->get_object_aabb != nullptr)
    {
        arkheon_vec3 minimum{};
        arkheon_vec3 maximum{};
        if (env->get_object_aabb(env->host_ctx, goal->target_object_id, &minimum, &maximum) != 0)
        {
            const arkheon_vec3 center{
                (minimum.x + maximum.x) * 0.5f,
                (minimum.y + maximum.y) * 0.5f,
                (minimum.z + maximum.z) * 0.5f
            };
            const float distance = length2D(subtract(center, currentRoot));
            if (distance > 0.65f)
            {
                const arkheon_vec3 direction = normalize2D(subtract(center, currentRoot));
                return {direction.x * 0.85f * dt, 0.0f, direction.z * 0.85f * dt};
            }

            if (goal->type == ARK_GOAL_PUSH && length2D(goal->push_dir) > 0.001f)
            {
                const arkheon_vec3 pushDirection = normalize2D(goal->push_dir);
                return {pushDirection.x * 0.25f * dt, 0.0f, pushDirection.z * 0.25f * dt};
            }

            if (env->report_goal_complete != nullptr)
            {
                env->report_goal_complete(env->host_ctx, goal->sequence_id, ARK_GOAL_RESULT_OK);
            }
        }
    }

    if (goal->type == ARK_GOAL_PICKUP || goal->type == ARK_GOAL_CLIMB)
    {
        const float tolerance = goal->tolerance_m > 0.0f ? goal->tolerance_m : 0.45f;
        const arkheon_vec3 toTarget = subtract(goal->target_position, currentRoot);
        const float distance = length2D(toTarget);
        if (distance > tolerance)
        {
            const arkheon_vec3 direction = normalize2D(toTarget);
            const float speed = goal->type == ARK_GOAL_CLIMB ? 0.75f : 0.65f;
            return {direction.x * speed * dt, 0.0f, direction.z * speed * dt};
        }

        if (env != nullptr && env->report_goal_complete != nullptr)
        {
            env->report_goal_complete(env->host_ctx, goal->sequence_id, ARK_GOAL_RESULT_OK);
        }

        if (goal->type == ARK_GOAL_CLIMB)
        {
            return {0.0f, 0.10f * dt, 0.0f};
        }
    }

    return {0.0f, 0.0f, 0.0f};
}

int motionForGoal(const arkheon_mission_goal* goal, int currentMotion)
{
    if (goal == nullptr)
    {
        return currentMotion;
    }

    switch (goal->type)
    {
    case ARK_GOAL_PUSH:
    case ARK_GOAL_INTERACT:
    case ARK_GOAL_PICKUP:
        return 1;
    case ARK_GOAL_CLIMB:
        return 2;
    default:
        return currentMotion;
    }
}

void applyTaskMotionLayer(int motion,
                          float time,
                          CharacterProject::JointOutput& output)
{
    using namespace CharacterProject;

    if (motion == 1)
    {
        const float pulse = 0.5f + (0.5f * std::sin(time * TwoPi * 0.55f));
        const float brace = 0.65f + (0.20f * pulse);
        output[SpinePitch] = blendAngle(output[SpinePitch], radiansFromDegrees(8.0f), 0.72f);
        output[LeftShoulderPitch] = blendAngle(output[LeftShoulderPitch], radiansFromDegrees(58.0f + 8.0f * pulse), 0.88f);
        output[RightShoulderPitch] = blendAngle(output[RightShoulderPitch], radiansFromDegrees(58.0f + 8.0f * pulse), 0.88f);
        output[LeftElbowPitch] = blendAngle(output[LeftElbowPitch], radiansFromDegrees(32.0f - 8.0f * pulse), 0.82f);
        output[RightElbowPitch] = blendAngle(output[RightElbowPitch], radiansFromDegrees(32.0f - 8.0f * pulse), 0.82f);
        output[LeftHipPitch] = blendAngle(output[LeftHipPitch], radiansFromDegrees(-16.0f), brace);
        output[RightHipPitch] = blendAngle(output[RightHipPitch], radiansFromDegrees(-16.0f), brace);
        output[LeftKneePitch] = blendAngle(output[LeftKneePitch], radiansFromDegrees(24.0f + 6.0f * pulse), brace);
        output[RightKneePitch] = blendAngle(output[RightKneePitch], radiansFromDegrees(24.0f + 6.0f * pulse), brace);
    }
    else if (motion == 2)
    {
        const float phase = time * TwoPi * 0.72f;
        const float leftLead = 0.5f + (0.5f * std::sin(phase));
        const float rightLead = 1.0f - leftLead;
        output[SpinePitch] = blendAngle(output[SpinePitch], radiansFromDegrees(12.0f), 0.65f);
        output[LeftShoulderPitch] = blendAngle(output[LeftShoulderPitch], radiansFromDegrees(35.0f + 35.0f * rightLead), 0.75f);
        output[RightShoulderPitch] = blendAngle(output[RightShoulderPitch], radiansFromDegrees(35.0f + 35.0f * leftLead), 0.75f);
        output[LeftElbowPitch] = blendAngle(output[LeftElbowPitch], radiansFromDegrees(40.0f + 24.0f * rightLead), 0.70f);
        output[RightElbowPitch] = blendAngle(output[RightElbowPitch], radiansFromDegrees(40.0f + 24.0f * leftLead), 0.70f);
        output[LeftHipPitch] = blendAngle(output[LeftHipPitch], radiansFromDegrees(52.0f * leftLead - 10.0f * rightLead), 0.82f);
        output[RightHipPitch] = blendAngle(output[RightHipPitch], radiansFromDegrees(52.0f * rightLead - 10.0f * leftLead), 0.82f);
        output[LeftKneePitch] = blendAngle(output[LeftKneePitch], radiansFromDegrees(20.0f + 72.0f * leftLead), 0.88f);
        output[RightKneePitch] = blendAngle(output[RightKneePitch], radiansFromDegrees(20.0f + 72.0f * rightLead), 0.88f);
    }

    for (int i = 0; i < JointCount; ++i)
    {
        if (!std::isfinite(output.angles[static_cast<std::size_t>(i)]))
        {
            output.angles[static_cast<std::size_t>(i)] = 0.0f;
        }
    }
}

CharacterProject::HumanState makeHumanState(Controller& controller,
                                            const arkheon_frame* frame,
                                            const arkheon_bone_state inBones[66],
                                            arkheon_vec3 rootDelta,
                                            const arkheon_input_state* input)
{
    CharacterProject::HumanState state;
    const float time = frame != nullptr ? static_cast<float>(finiteOr(static_cast<float>(frame->simulation_time_s), 0.0f)) : 0.0f;
    state.timeSeconds = time;
    state.desiredSpeed = length2D(rootDelta) / FixedDt;
    state.rootPosition = toVec3(rootPositionFromBones(inBones));
    state.rootVelocity = CharacterProject::Vec3(rootDelta.x / FixedDt, rootDelta.y / FixedDt, rootDelta.z / FixedDt);

    if (inBones != nullptr)
    {
        state.leftFootPosition = toVec3(inBones[58].world_position);
        state.rightFootPosition = toVec3(inBones[63].world_position);
    }

    const float phase = std::fmod(time * 2.0f * Pi * 1.35f, 2.0f * Pi);
    const bool leftContact = std::sin(phase) < 0.0f;
    state.contacts.leftFootInContact = leftContact;
    state.contacts.rightFootInContact = !leftContact;
    state.contacts.leftFootWeight = leftContact ? 0.9f : 0.15f;
    state.contacts.rightFootWeight = leftContact ? 0.15f : 0.9f;
    state.contacts.leftFootPosition = state.leftFootPosition;
    state.contacts.rightFootPosition = state.rightFootPosition;

    if (input != nullptr && input->hotkey_motion_b != 0)
    {
        state.contacts.leftFootInContact = true;
        state.contacts.rightFootInContact = true;
        state.contacts.leftFootWeight = 0.5f;
        state.contacts.rightFootWeight = 0.5f;
    }

    state.currentAngles = controller.jointOutput.angles;
    return state;
}

void writeReferenceOverrides(const CharacterProject::JointOutput& source, arkheon_bone_override outOverrides[10])
{
    const float shoulderL = source.angles[CharacterProject::LeftShoulderPitch];
    const float shoulderR = source.angles[CharacterProject::RightShoulderPitch];
    const float elbowL = source.angles[CharacterProject::LeftElbowPitch];
    const float elbowR = source.angles[CharacterProject::RightElbowPitch];
    const float hipL = source.angles[CharacterProject::LeftHipPitch];
    const float hipR = source.angles[CharacterProject::RightHipPitch];
    const float kneeL = source.angles[CharacterProject::LeftKneePitch];
    const float kneeR = source.angles[CharacterProject::RightKneePitch];
    const float spine = source.angles[CharacterProject::SpinePitch];

    outOverrides[ARK_JOINT_UPPERARM_L] = {multiply(axisAngle(1.0f, 0.0f, 0.0f, shoulderL), axisAngle(0.0f, 0.0f, 1.0f, 0.10f + spine * 0.20f)), 1};
    outOverrides[ARK_JOINT_UPPERARM_R] = {multiply(axisAngle(1.0f, 0.0f, 0.0f, shoulderR), axisAngle(0.0f, 0.0f, 1.0f, -0.10f - spine * 0.20f)), 1};
    outOverrides[ARK_JOINT_LOWERARM_L] = {axisAngle(1.0f, 0.0f, 0.0f, elbowL), 1};
    outOverrides[ARK_JOINT_LOWERARM_R] = {axisAngle(1.0f, 0.0f, 0.0f, elbowR), 1};
    outOverrides[ARK_JOINT_THIGH_L] = {axisAngle(1.0f, 0.0f, 0.0f, hipL), 1};
    outOverrides[ARK_JOINT_THIGH_R] = {axisAngle(1.0f, 0.0f, 0.0f, hipR), 1};
    outOverrides[ARK_JOINT_CALF_L] = {axisAngle(1.0f, 0.0f, 0.0f, -kneeL), 1};
    outOverrides[ARK_JOINT_CALF_R] = {axisAngle(1.0f, 0.0f, 0.0f, -kneeR), 1};
    outOverrides[ARK_JOINT_FOOT_L] = {axisAngle(1.0f, 0.0f, 0.0f, clampValue(kneeL * 0.22f - hipL * 0.18f, -0.45f, 0.45f)), 1};
    outOverrides[ARK_JOINT_FOOT_R] = {axisAngle(1.0f, 0.0f, 0.0f, clampValue(kneeR * 0.22f - hipR * 0.18f, -0.45f, 0.45f)), 1};
}

void applyJointPhysics(Controller& controller,
                       const arkheon_bone_override reference[ARK_JOINT_COUNT],
                       arkheon_bone_override outOverrides[ARK_JOINT_COUNT],
                       float dt)
{
    if (!controller.physicsInitialized)
    {
        for (int i = 0; i < ARK_JOINT_COUNT; ++i)
        {
            controller.simulatedRotation[i] = normalizeQuat(reference[i].local_rotation);
            controller.angularVelocity[i] = {0.0f, 0.0f, 0.0f};
        }
        controller.physicsInitialized = true;
    }

    for (int i = 0; i < ARK_JOINT_COUNT; ++i)
    {
        const arkheon_quat target = normalizeQuat(reference[i].local_rotation);
        const arkheon_quat current = normalizeQuat(controller.simulatedRotation[i]);
        const arkheon_vec3 error = shortestArcError(target, current);
        const JointGains gains = gainsForJoint(i);
        const float inertia = std::max(0.01f, inertiaForJoint(controller, i));
        const arkheon_vec3 damping = multiplyVec(controller.angularVelocity[i], gains.kd);
        const arkheon_vec3 torque = subtract(multiplyVec(error, gains.kp), damping);
        arkheon_vec3 angularAcceleration = multiplyVec(torque, 1.0f / inertia);

        const float accelerationLength = length3D(angularAcceleration);
        if (accelerationLength > 120.0f)
        {
            angularAcceleration = multiplyVec(angularAcceleration, 120.0f / accelerationLength);
        }

        controller.angularVelocity[i] = add(controller.angularVelocity[i], multiplyVec(angularAcceleration, dt));
        const float speed = length3D(controller.angularVelocity[i]);
        if (!std::isfinite(speed))
        {
            controller.angularVelocity[i] = {0.0f, 0.0f, 0.0f};
        }
        else if (speed > 18.0f)
        {
            controller.angularVelocity[i] = multiplyVec(controller.angularVelocity[i], 18.0f / speed);
        }

        controller.simulatedRotation[i] = integrateOrientation(current, controller.angularVelocity[i], dt);
        outOverrides[i].local_rotation = controller.simulatedRotation[i];
        outOverrides[i].apply = 1;
    }
}
}

#ifdef _WIN32
BOOL APIENTRY DllMain(HMODULE module, DWORD reason, LPVOID reserved)
{
    (void)module;
    (void)reserved;

    switch (reason)
    {
    case DLL_PROCESS_ATTACH:
        traceExport("DllMain", "entry DLL_PROCESS_ATTACH");
        traceExport("DllMain", "exit DLL_PROCESS_ATTACH");
        break;
    case DLL_PROCESS_DETACH:
        traceExport("DllMain", "entry DLL_PROCESS_DETACH");
        traceExport("DllMain", "exit DLL_PROCESS_DETACH");
        break;
    case DLL_THREAD_ATTACH:
        traceExport("DllMain", "entry DLL_THREAD_ATTACH");
        traceExport("DllMain", "exit DLL_THREAD_ATTACH");
        break;
    case DLL_THREAD_DETACH:
        traceExport("DllMain", "entry DLL_THREAD_DETACH");
        traceExport("DllMain", "exit DLL_THREAD_DETACH");
        break;
    default:
        traceExport("DllMain", "entry UNKNOWN");
        traceExport("DllMain", "exit UNKNOWN");
        break;
    }

    return TRUE;
}
#endif

extern "C" {

ARKHEON_CHAR_EXPORT const char* get_plugin_signature(void)
{
    traceExport("get_plugin_signature", "entry");
    const char* signature = "ARKHEON_PLUGIN_V1";
    traceExport("get_plugin_signature", "exit");
    return signature;
}

ARKHEON_CHAR_EXPORT void* create_plugin(void)
{
    traceExport("create_plugin", "entry");
    try
    {
        void* handle = new N8roGenericPluginHandle{0x43484152u};
        traceExport("create_plugin", "exit");
        return handle;
    }
    catch (...)
    {
        traceExport("create_plugin", "exit null exception");
        return nullptr;
    }
}

ARKHEON_CHAR_EXPORT void destroy_plugin(void* handle)
{
    traceExport("destroy_plugin", "entry");
    delete static_cast<N8roGenericPluginHandle*>(handle);
    traceExport("destroy_plugin", "exit");
}

ARKHEON_CHAR_EXPORT int32_t plugin_tick(void* handle, double delta_time_s)
{
    traceExport("plugin_tick", "entry");
    (void)handle;
    (void)delta_time_s;
    traceExportStatus("plugin_tick", "exit", 0);
    return 0;
}

ARKHEON_CHAR_EXPORT uint32_t arkheon_character_sdk_version(void)
{
    traceExport("arkheon_character_sdk_version", "entry");
    const uint32_t version = ARKHEON_CHARACTER_SDK_VERSION;
    traceExport("arkheon_character_sdk_version", "exit");
    return version;
}

ARKHEON_CHAR_EXPORT const char* arkheon_character_plugin_name(void)
{
    traceExport("arkheon_character_plugin_name", "entry");
    const char* name = "Ugurkan Character Controller v1.0";
    traceExport("arkheon_character_plugin_name", "exit");
    return name;
}

ARKHEON_CHAR_EXPORT void arkheon_character_get_motion_clips(void* handle, int32_t out_clip_ids[3])
{
    traceExport("arkheon_character_get_motion_clips", "entry");
    (void)handle;
    if (out_clip_ids == nullptr)
    {
        traceExport("arkheon_character_get_motion_clips", "exit null output");
        return;
    }

    out_clip_ids[0] = 12;
    out_clip_ids[1] = 47;
    out_clip_ids[2] = 83;
    traceExport("arkheon_character_get_motion_clips", "exit");
}

ARKHEON_CHAR_EXPORT void* arkheon_character_create(const float segment_lengths_m[10])
{
    traceExport("arkheon_character_create", "entry");
    try
    {
        Controller* controller = new Controller();
        if (segment_lengths_m != nullptr)
        {
            std::memcpy(controller->segmentLengths, segment_lengths_m, sizeof(controller->segmentLengths));
        }
        traceExport("arkheon_character_create", "exit");
        return controller;
    }
    catch (...)
    {
        traceExport("arkheon_character_create", "exit null exception");
        return nullptr;
    }
}

ARKHEON_CHAR_EXPORT void arkheon_character_destroy(void* handle)
{
    traceExport("arkheon_character_destroy", "entry");
    Controller* controller = static_cast<Controller*>(handle);
    delete controller;
    traceExport("arkheon_character_destroy", "exit");
}

ARKHEON_CHAR_EXPORT int32_t arkheon_character_tick(
    void* handle,
    const arkheon_frame* frame,
    const arkheon_bone_state in_bones[66],
    arkheon_bone_override out_overrides[10],
    arkheon_vec3* out_root_translation_delta,
    arkheon_quat* out_root_rotation_delta,
    const arkheon_input_state* input,
    const arkheon_mission_goal* current_goal,
    const arkheon_env_api* env)
{
    traceExport("arkheon_character_tick", "entry");
    try
    {
        Controller* controller = static_cast<Controller*>(handle);
        if (controller == nullptr || out_overrides == nullptr ||
            out_root_translation_delta == nullptr || out_root_rotation_delta == nullptr)
        {
            traceExportStatus("arkheon_character_tick", "exit", 1);
            return 1;
        }

        if (frame != nullptr && frame->is_paused != 0)
        {
            for (int i = 0; i < ARK_JOINT_COUNT; ++i)
            {
                out_overrides[i] = controller->lastOverrides[i];
            }
            *out_root_translation_delta = {0.0f, 0.0f, 0.0f};
            *out_root_rotation_delta = controller->lastRootRotation;
            traceExportStatus("arkheon_character_tick", "exit", 0);
            return 0;
        }

        updateMotionHotkeys(*controller, input);

        arkheon_vec3 rootDelta = computeManualLocomotion(input, FixedDt, controller->rootYawRad);
        const arkheon_vec3 currentRoot = rootPositionFromBones(in_bones);
        const arkheon_vec3 goalDelta = computeGoalLocomotion(*controller, current_goal, env, currentRoot, FixedDt);
        if (current_goal != nullptr && current_goal->type != ARK_GOAL_NONE)
        {
            rootDelta = goalDelta;
        }

        controller->humanState = makeHumanState(*controller, frame, in_bones, rootDelta, input);
        if (controller->activeMotion == 1)
        {
            controller->humanState.desiredSpeed = std::max(controller->humanState.desiredSpeed, 0.15f);
        }
        else if (controller->activeMotion == 2)
        {
            controller->humanState.desiredSpeed = std::max(controller->humanState.desiredSpeed, 0.55f);
        }

        controller->jointController.update(FixedDt, controller->humanState, controller->jointOutput);
        const int effectiveMotion = motionForGoal(current_goal, controller->activeMotion);
        const float motionTime = frame != nullptr ? static_cast<float>(finiteOr(static_cast<float>(frame->simulation_time_s), 0.0f)) : 0.0f;
        applyTaskMotionLayer(effectiveMotion, motionTime, controller->jointOutput);

        arkheon_bone_override referenceOverrides[ARK_JOINT_COUNT] = {};
        writeReferenceOverrides(controller->jointOutput, referenceOverrides);
        applyJointPhysics(*controller, referenceOverrides, out_overrides, FixedDt);

        const float yaw = finiteOr(input != nullptr ? input->look_yaw_rad : controller->rootYawRad, controller->rootYawRad);
        controller->rootYawRad = yaw;
        *out_root_translation_delta = rootDelta;
        *out_root_rotation_delta = yawQuat(yaw * 0.04f);

        for (int i = 0; i < ARK_JOINT_COUNT; ++i)
        {
            out_overrides[i].local_rotation = normalizeQuat(out_overrides[i].local_rotation);
            out_overrides[i].apply = 1;
            controller->lastOverrides[i] = out_overrides[i];
        }
        controller->lastRootTranslation = *out_root_translation_delta;
        controller->lastRootRotation = *out_root_rotation_delta;
        traceExportStatus("arkheon_character_tick", "exit", 0);
        return 0;
    }
    catch (...)
    {
        traceExportStatus("arkheon_character_tick", "exit", 2);
        return 2;
    }
}

}
