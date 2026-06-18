#include "arkheon/character/ICharacterController.h"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>

static int32_t mock_raycast(void*, arkheon_vec3, arkheon_vec3, float,
                            arkheon_vec3* hit, arkheon_vec3* normal, int32_t* id)
{
    *hit = {0.0f, 0.0f, 0.0f};
    *normal = {0.0f, 1.0f, 0.0f};
    *id = -1;
    return 0;
}

static int32_t mock_aabb(void*, int32_t, arkheon_vec3* minimum, arkheon_vec3* maximum)
{
    *minimum = {-1.0f, -1.0f, -1.0f};
    *maximum = {1.0f, 1.0f, 1.0f};
    return 1;
}

static int32_t mock_find(void*, const char*)
{
    return 0;
}

static int32_t mock_nav(void*, arkheon_vec3, arkheon_vec3 to, arkheon_vec3* path, int32_t maxPoints)
{
    if (path == nullptr || maxPoints <= 0)
    {
        return 0;
    }

    path[0] = to;
    return 1;
}

static void mock_done(void*, int32_t sequenceId, int32_t result)
{
    std::printf("goal %d done, result=%d\n", sequenceId, result);
}

static arkheon_vec3 mock_gravity(void*)
{
    return {0.0f, -9.81f, 0.0f};
}

static bool is_unitish(arkheon_quat q)
{
    const float n2 = (q.x * q.x) + (q.y * q.y) + (q.z * q.z) + (q.w * q.w);
    return std::isfinite(n2) && n2 > 0.5f && n2 < 1.5f;
}

#ifdef _WIN32
static bool verify_dll_exports()
{
    HMODULE library = LoadLibraryA("character_plugin_ugurkan.dll");
    if (library == nullptr)
    {
        std::printf("FAIL: LoadLibrary(character_plugin_ugurkan.dll) error=%lu\n", GetLastError());
        return false;
    }

    const char* requiredExports[] = {
        "arkheon_character_sdk_version",
        "arkheon_character_plugin_name",
        "arkheon_character_get_motion_clips",
        "arkheon_character_create",
        "arkheon_character_destroy",
        "arkheon_character_tick",
        "get_plugin_signature",
        "create_plugin",
        "destroy_plugin",
        "plugin_tick"
    };

    for (const char* exportName : requiredExports)
    {
        if (GetProcAddress(library, exportName) == nullptr)
        {
            std::printf("FAIL: missing export %s\n", exportName);
            FreeLibrary(library);
            return false;
        }
    }

    using VersionFn = uint32_t (*)(void);
    using SignatureFn = const char* (*)(void);
    using CreatePluginFn = void* (*)(void);
    using TickPluginFn = int32_t (*)(void*, double);
    using DestroyPluginFn = void (*)(void*);

    const auto versionFn = reinterpret_cast<VersionFn>(GetProcAddress(library, "arkheon_character_sdk_version"));
    const auto signatureFn = reinterpret_cast<SignatureFn>(GetProcAddress(library, "get_plugin_signature"));
    const auto createPluginFn = reinterpret_cast<CreatePluginFn>(GetProcAddress(library, "create_plugin"));
    const auto tickPluginFn = reinterpret_cast<TickPluginFn>(GetProcAddress(library, "plugin_tick"));
    const auto destroyPluginFn = reinterpret_cast<DestroyPluginFn>(GetProcAddress(library, "destroy_plugin"));

    if (versionFn() != ARKHEON_CHARACTER_SDK_VERSION)
    {
        std::printf("FAIL: DLL SDK version mismatch\n");
        FreeLibrary(library);
        return false;
    }

    const char* signature = signatureFn();
    if (signature == nullptr || std::strcmp(signature, "ARKHEON_PLUGIN_V1") != 0)
    {
        std::printf("FAIL: generic plugin signature mismatch\n");
        FreeLibrary(library);
        return false;
    }

    void* genericHandle = createPluginFn();
    if (genericHandle == nullptr)
    {
        std::printf("FAIL: generic create_plugin returned null\n");
        FreeLibrary(library);
        return false;
    }

    const int32_t genericTick = tickPluginFn(genericHandle, 0.02);
    destroyPluginFn(genericHandle);
    if (genericTick != 0)
    {
        std::printf("FAIL: generic plugin_tick rc=%d\n", genericTick);
        FreeLibrary(library);
        return false;
    }

    FreeLibrary(library);
    std::printf("DLL export/load check passed\n");
    return true;
}
#endif

int main()
{
#ifdef _WIN32
    if (!verify_dll_exports())
    {
        return 1;
    }
#endif

    std::printf("SDK version = 0x%08x\n", arkheon_character_sdk_version());
    if (arkheon_character_sdk_version() != ARKHEON_CHARACTER_SDK_VERSION)
    {
        std::printf("FAIL: SDK version mismatch\n");
        return 1;
    }

    int32_t clips[3] = {};
    arkheon_character_get_motion_clips(nullptr, clips);
    std::printf("clips = %d, %d, %d\n", clips[0], clips[1], clips[2]);

    float segments[10] = {
        0.0f, 0.0f, 0.296595f, 0.296595f, 0.0f,
        0.0f, 0.406626f, 0.406626f, 0.433194f, 0.433194f
    };
    void* handle = arkheon_character_create(segments);
    if (handle == nullptr)
    {
        std::printf("FAIL: create\n");
        return 1;
    }

    arkheon_bone_state bones[66] = {};
    for (arkheon_bone_state& bone : bones)
    {
        bone.local_rotation = {0.0f, 0.0f, 0.0f, 1.0f};
    }
    bones[0].world_position = {0.0f, 1.0f, 0.0f};
    bones[58].world_position = {-0.10f, 0.0f, 0.05f};
    bones[63].world_position = {0.10f, 0.0f, -0.05f};

    arkheon_input_state input = {};
    arkheon_env_api env = {
        nullptr,
        mock_raycast,
        mock_aabb,
        mock_find,
        mock_nav,
        mock_done,
        mock_gravity
    };

    arkheon_frame frame = {};
    frame.delta_time_s = 0.02;
    arkheon_vec3 accumulatedRoot = {0.0f, 0.0f, 0.0f};

    const auto start = std::chrono::high_resolution_clock::now();
    for (uint64_t i = 0; i < 1000; ++i)
    {
        frame.simulation_time_s = static_cast<double>(i) * 0.02;
        frame.frame_number = i;
        input.keys[26] = i < 200 ? 1 : 0;
        input.look_yaw_rad = 0.15f;
        input.hotkey_motion_a = i == 10 ? 1 : 0;
        input.hotkey_motion_b = i == 300 ? 1 : 0;
        input.hotkey_motion_c = i == 600 ? 1 : 0;

        arkheon_bone_override overrides[10] = {};
        arkheon_vec3 rootDelta = {0.0f, 0.0f, 0.0f};
        arkheon_quat rootRotationDelta = {0.0f, 0.0f, 0.0f, 1.0f};
        const int32_t rc = arkheon_character_tick(
            handle,
            &frame,
            bones,
            overrides,
            &rootDelta,
            &rootRotationDelta,
            &input,
            nullptr,
            &env);
        if (rc != 0)
        {
            std::printf("FAIL: tick %llu rc=%d\n", static_cast<unsigned long long>(i), rc);
            arkheon_character_destroy(handle);
            return 1;
        }

        for (const arkheon_bone_override& overrideValue : overrides)
        {
            if (overrideValue.apply != 1 || !is_unitish(overrideValue.local_rotation))
            {
                std::printf("FAIL: bad override at tick %llu\n", static_cast<unsigned long long>(i));
                arkheon_character_destroy(handle);
                return 1;
            }
        }

        if (!is_unitish(rootRotationDelta))
        {
            std::printf("FAIL: bad root quaternion at tick %llu\n", static_cast<unsigned long long>(i));
            arkheon_character_destroy(handle);
            return 1;
        }

        accumulatedRoot.x += rootDelta.x;
        accumulatedRoot.y += rootDelta.y;
        accumulatedRoot.z += rootDelta.z;
        bones[0].world_position = {accumulatedRoot.x, 1.0f, accumulatedRoot.z};
    }
    const auto end = std::chrono::high_resolution_clock::now();

    const double elapsedMs = std::chrono::duration<double, std::milli>(end - start).count();
    const double averageTickMs = elapsedMs / 1000.0;
    std::printf("PASS: 1000 ticks, all quaternions finite & unit-ish\n");
    std::printf("Average tick time: %.6f ms\n", averageTickMs);

    arkheon_character_destroy(handle);
    return averageTickMs < 18.0 ? 0 : 1;
}
