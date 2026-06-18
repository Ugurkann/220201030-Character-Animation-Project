#pragma once

#include <stdint.h>

#define ARKHEON_CHARACTER_SDK_VERSION 0x00010000

#ifdef _WIN32
#define ARKHEON_CHAR_EXPORT __declspec(dllexport)
#else
#define ARKHEON_CHAR_EXPORT __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct { float x, y, z; } arkheon_vec3;
typedef struct { float x, y, z, w; } arkheon_quat;

typedef struct {
    arkheon_quat local_rotation;
    arkheon_vec3 local_translation;
    arkheon_vec3 world_position;
} arkheon_bone_state;

typedef enum {
    ARK_JOINT_UPPERARM_L = 0, ARK_JOINT_UPPERARM_R = 1,
    ARK_JOINT_LOWERARM_L = 2, ARK_JOINT_LOWERARM_R = 3,
    ARK_JOINT_THIGH_L    = 4, ARK_JOINT_THIGH_R    = 5,
    ARK_JOINT_CALF_L     = 6, ARK_JOINT_CALF_R     = 7,
    ARK_JOINT_FOOT_L     = 8, ARK_JOINT_FOOT_R     = 9,
    ARK_JOINT_COUNT      = 10
} arkheon_major_joint;

typedef struct {
    arkheon_quat local_rotation;
    uint8_t apply;
} arkheon_bone_override;

typedef struct {
    double simulation_time_s;
    double delta_time_s;
    uint64_t frame_number;
    uint8_t is_paused;
} arkheon_frame;

typedef struct {
    uint8_t keys[256];
    float mouse_dx;
    float mouse_dy;
    float look_yaw_rad;
    float look_pitch_rad;
    uint8_t mouse_buttons;
    uint8_t hotkey_motion_a;
    uint8_t hotkey_motion_b;
    uint8_t hotkey_motion_c;
} arkheon_input_state;

typedef enum {
    ARK_GOAL_NONE     = 0,
    ARK_GOAL_GOTO     = 1,
    ARK_GOAL_PUSH     = 2,
    ARK_GOAL_CLIMB    = 3,
    ARK_GOAL_PICKUP   = 4,
    ARK_GOAL_INTERACT = 5
} arkheon_goal_type;

typedef struct {
    int32_t sequence_id;
    arkheon_goal_type type;
    arkheon_vec3 target_position;
    int32_t target_object_id;
    arkheon_vec3 push_dir;
    float tolerance_m;
    float timeout_s;
} arkheon_mission_goal;

#define ARK_GOAL_RESULT_OK 0
#define ARK_GOAL_RESULT_FAIL 1
#define ARK_GOAL_RESULT_TIMEOUT 2

typedef struct {
    void* host_ctx;

    int32_t (*raycast)(void* ctx,
                       arkheon_vec3 origin,
                       arkheon_vec3 dir_normalized,
                       float max_dist_m,
                       arkheon_vec3* out_hit_world,
                       arkheon_vec3* out_normal,
                       int32_t* out_object_id);

    int32_t (*get_object_aabb)(void* ctx, int32_t object_id,
                               arkheon_vec3* out_min,
                               arkheon_vec3* out_max);

    int32_t (*find_object_by_name)(void* ctx, const char* name);

    int32_t (*navmesh_query)(void* ctx,
                             arkheon_vec3 from, arkheon_vec3 to,
                             arkheon_vec3* out_path,
                             int32_t max_pts);

    void (*report_goal_complete)(void* ctx, int32_t sequence_id, int32_t result);

    arkheon_vec3 (*get_gravity)(void* ctx);
} arkheon_env_api;

ARKHEON_CHAR_EXPORT uint32_t arkheon_character_sdk_version(void);
ARKHEON_CHAR_EXPORT const char* arkheon_character_plugin_name(void);

ARKHEON_CHAR_EXPORT void arkheon_character_get_motion_clips(
    void* handle, int32_t out_clip_ids[3]);

ARKHEON_CHAR_EXPORT void* arkheon_character_create(
    const float segment_lengths_m[10]);

ARKHEON_CHAR_EXPORT void arkheon_character_destroy(void* handle);

ARKHEON_CHAR_EXPORT int32_t arkheon_character_tick(
    void* handle,
    const arkheon_frame* frame,
    const arkheon_bone_state in_bones[66],
    arkheon_bone_override out_overrides[10],
    arkheon_vec3* out_root_translation_delta,
    arkheon_quat* out_root_rotation_delta,
    const arkheon_input_state* input,
    const arkheon_mission_goal* current_goal,
    const arkheon_env_api* env);

#ifdef __cplusplus
}
#endif
