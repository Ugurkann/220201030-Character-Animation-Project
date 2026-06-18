local animationCode = "Knee Lift"
local healthLogIntervalSeconds = 2.0

local stateByEntity = {}

local function missionLog(message)
    if mission and mission.log then
        mission.log("[human_animation_loop] " .. message)
    end
end

local function requestAnimation(entityId, state, reason, simulationTimeSeconds)
    local code = animationCode
    local ok = false

    if animation and animation.setAnimation then
        ok = animation.setAnimation(entityId, code, true, 1.0) or false
    end

    state.currentCode = code
    state.lastRequestTime = simulationTimeSeconds or 0.0

    missionLog(string.format(
        "entity=%s reason=%s code=%s ok=%s t=%.2f",
        tostring(entityId),
        tostring(reason),
        tostring(code),
        tostring(ok),
        tonumber(simulationTimeSeconds or 0.0)))

    return ok
end

function onInit(entityId)
    local state = {
        currentCode = animationCode,
        lastHealthLogTime = -healthLogIntervalSeconds,
        lastRequestTime = 0.0
    }

    stateByEntity[entityId] = state

    if mission and mission.markRunning then
        mission.markRunning(entityId)
    end

    requestAnimation(entityId, state, "init", 0.0)
end

function onTick(entityId, simulationTimeSeconds, deltaTimeSeconds)
    local state = stateByEntity[entityId]
    if not state then
        onInit(entityId)
        state = stateByEntity[entityId]
    end

    local desiredCode = state.currentCode or animationCode
    local activeCode = nil

    if animation and animation.getActiveAnimation then
        activeCode = animation.getActiveAnimation(entityId)
    end

    if activeCode ~= desiredCode then
        requestAnimation(entityId, state, "resync", simulationTimeSeconds)
        activeCode = state.currentCode
    end

    if (simulationTimeSeconds - (state.lastHealthLogTime or 0.0)) >= healthLogIntervalSeconds then
        state.lastHealthLogTime = simulationTimeSeconds
        if animation and animation.getActiveAnimation then
            activeCode = animation.getActiveAnimation(entityId)
        end

        missionLog(string.format(
            "entity=%s desired=%s active=%s dt=%.3f t=%.2f",
            tostring(entityId),
            tostring(state.currentCode),
            tostring(activeCode),
            tonumber(deltaTimeSeconds or 0.0),
            tonumber(simulationTimeSeconds or 0.0)))
    end
end

function onShutdown(entityId)
    stateByEntity[entityId] = nil
end
