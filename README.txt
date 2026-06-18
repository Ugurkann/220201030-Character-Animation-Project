CharacterProject Arkheon Plugin
===============================

Overview
--------
This project builds a Windows x64 C++17 DLL that implements the official Arkheon Character Animation Integration Specification v1.0 ABI. The Release DLL is named:

    character_plugin_ugurkan.dll

The plugin preserves the existing procedural human motion controller and adapts it to the official SDK boundary by outputting ten quaternion-safe major-joint overrides plus root transform deltas. A lightweight per-joint PD layer tracks the procedural references with simple angular rigid-body integration.

Official ABI
------------
The SDK header from Section 5 of the specification is provided at:

    include/arkheon/character/ICharacterController.h

The DLL exports exactly these functions:

    arkheon_character_sdk_version
    arkheon_character_plugin_name
    arkheon_character_get_motion_clips
    arkheon_character_create
    arkheon_character_destroy
    arkheon_character_tick

The previous local exports createController, destroyController, initializeController, resetController, and updateController are no longer exported by the submitted DLL.

Motion Clips
------------
The plugin reports three motion clips to the host:

- Q / Motion A: clip 12, walk_forward
- E / Motion B: clip 47, push_two_handed
- R / Motion C: clip 83, climb_low_step

Q keeps the balanced locomotion controller in its walk/idle mode. E layers a braced two-hand push pose over the current gait so the shoulders, elbows, hips, and knees visibly prepare for contact. R layers an alternating climb pose with raised knees and reaching arms. These layers are implemented as original procedural targets before the PD/RBD tracking stage, so the exported joint rotations remain quaternion-based and dynamically smoothed.

Controller Behavior
-------------------
The tick function runs with fixed 50 Hz timing using the specification timestep of 0.02 seconds. Input handling reads USB HID scancodes for WASD locomotion and left-shift sprint. Root translation is aligned to host look yaw. Root rotation output is a normalized yaw quaternion.

The internal controller continues to generate Center-of-Mass-aware, contact-informed procedural motion. Its scalar joint angles are mapped into target quaternions for the official ten Arkheon major joints:

- upperarm_l, upperarm_r
- lowerarm_l, lowerarm_r
- thigh_l, thigh_r
- calf_l, calf_r
- foot_l, foot_r

Physics Approximation
---------------------
Each major joint maintains persistent angular velocity and simulated local orientation state. Every tick computes a shortest-arc quaternion error from the simulated orientation to the procedural reference orientation. The controller applies:

    torque = Kp * orientation_error - Kd * angular_velocity

The torque is divided by a simple inertia estimate based on segment_lengths_m[10] where available, then integrated into angular velocity and local quaternion orientation. This is a lightweight rigid-body angular approximation suitable for the 10-joint SDK output. It is not a full articulated rigid-body solver, but it addresses the physics-quality requirement by adding per-joint inertial lag, damping, and stable PD tracking.

PD gains:

- Upper arms: Kp 46, Kd 8.5
- Lower arms / elbows: Kp 58, Kd 9.0
- Thighs: Kp 70, Kd 11.5
- Calves / knees: Kp 64, Kd 10.0
- Feet: Kp 38, Kd 7.0

All output quaternions are normalized before crossing the ABI. Angular acceleration and angular velocity are clamped to protect against numerical spikes.

Mission Handling
----------------
The plugin tracks mission goal sequence IDs. For GOTO goals it requests a path through env->navmesh_query when available, walks toward the active waypoint, and reports ARK_GOAL_RESULT_OK when the root reaches the specified tolerance. PUSH and INTERACT approach the object AABB, then apply a small kinematic push delta or report success once the character is close enough. PICKUP approaches the goal and reports success at tolerance. CLIMB approaches the target, activates the climb motion layer, and emits a small upward root delta after arrival. These are lightweight mission skeletons, not game-specific task solvers, but every official goal type is handled without crashing or producing invalid output.

APP / N8RO Loader Diagnostics
-----------------------------
The Release DLL also includes loader-debug instrumentation for APP/N8RO integration. Every exported function logs entry and exit through OutputDebugStringA and appends the same text to:

    C:\N8RO\userPlugins\sim\character_plugin_debug.log

The generic APP loader shim exports are present in addition to the official character ABI:

    get_plugin_signature
    create_plugin
    destroy_plugin
    plugin_tick

get_plugin_signature returns ARKHEON_PLUGIN_V1. The generic shim is intentionally small because the official assignment ABI is the C character-controller interface. If N8RO expects a full proprietary APP IPlugin C++ object, the log file will show whether the loader stops after signature validation, generic creation, or the official character calls.

Build
-----
From a Visual Studio 2022 x64 Developer Command Prompt:

    cmake -S . -B build-vs2022 -G "Visual Studio 17 2022" -A x64
    cmake --build build-vs2022 --config Release

Outputs:

    out/character_plugin_ugurkan.dll
    out/CharacterProjectTest.exe

Standalone Test
---------------
The standalone test is based on Section 13 of the specification. It:

- Loads out/character_plugin_ugurkan.dll with LoadLibrary on Windows.
- Checks all official and loader-debug exports with GetProcAddress.
- Calls get_plugin_signature, create_plugin, plugin_tick, and destroy_plugin.
- Checks the SDK version handshake.
- Creates the controller through arkheon_character_create.
- Ticks the plugin 1000 times at 50 Hz.
- Exercises hotkey switching and WASD input.
- Verifies all ten joint overrides are applied.
- Verifies all joint and root quaternions are finite and unit-ish.
- Measures average tick time and requires it to stay below 18 ms.

Run:

    out\CharacterProjectTest.exe

Limitations
-----------
This implementation does not implement a full articulated-body or constraint-based RBD solver. The physics layer is a per-joint angular PD approximation with segment-length-scaled inertia. It satisfies the official ABI and provides stable quaternion overrides, locomotion, motion switching, mission-goal skeleton behavior, and offline validation. Full APP mission scoring can be improved by adding deeper PUSH, CLIMB, and PICKUP interaction logic using object state from the host callbacks.
