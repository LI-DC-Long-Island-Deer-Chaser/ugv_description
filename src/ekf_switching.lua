-- ekf_switching.lua
-- Always auto-switch between:
--   Source 1 (index 0) = GPS
--   Source 2 (index 1) = Non-GPS
--
-- Hardcoded thresholds, no SCR_USER params needed.

---@diagnostic disable: need-check-nil

-- =========================
-- Hardcoded config
-- =========================
local GPS_SOURCE            = 0     -- EK3_SRC1_*
local NONGPS_SOURCE         = 1     -- EK3_SRC2_*
local GPS_FIX_3D            = 3

local GPS_SPEED_ACC_THRESH  = 0.30  -- m/s
local GPS_INNOV_THRESH      = 0.30  -- innovation threshold
local VOTE_MAX              = 20    -- 20 * 100ms = 2s hysteresis
local UPDATE_MS             = 100

-- =========================
-- State
-- =========================
local vote = 0
local current_source = -1

-- =========================
-- Helpers
-- =========================
local function play_tune(source)
    if source == GPS_SOURCE then
        notify:play_tune("L8C")
    else
        notify:play_tune("L12DD")
    end
end

local function set_source(source, msg)
    if source ~= current_source then
        current_source = source
        ahrs:set_posvelyaw_source_set(source)
        gcs:send_text(6, msg)
        play_tune(source)
    end
end

local function gps_is_good()
    local inst = gps:primary_sensor()
    if inst == nil then
        return false
    end

    local status = gps:status(inst)
    local sacc   = gps:speed_accuracy(inst)
    local innov  = ahrs:get_vel_innovations_and_variances_for_source(3)

    local fix_ok   = (status ~= nil) and (status >= GPS_FIX_3D)
    local sacc_ok  = (sacc ~= nil) and (sacc <= GPS_SPEED_ACC_THRESH)
    local innov_ok = (innov ~= nil) and (innov:z() ~= 0.0) and (math.abs(innov:z()) <= GPS_INNOV_THRESH)

    return fix_ok and sacc_ok and innov_ok
end

-- =========================
-- Main loop
-- =========================
function update()
    local gps_good = gps_is_good()

    -- startup decision
    if current_source < 0 then
        if gps_good then
            vote = -VOTE_MAX
            set_source(GPS_SOURCE, "Auto: start on GPS")
        else
            vote = VOTE_MAX
            set_source(NONGPS_SOURCE, "Auto: start on Non-GPS")
        end
        return update, UPDATE_MS
    end

    -- hysteresis voting
    if gps_good then
        vote = math.max(vote - 1, -VOTE_MAX)
    else
        vote = math.min(vote + 1, VOTE_MAX)
    end

    -- switch only after enough consistent votes
    if vote <= -VOTE_MAX then
        set_source(GPS_SOURCE, "Auto: switched to GPS")
    elseif vote >= VOTE_MAX then
        set_source(NONGPS_SOURCE, "Auto: switched to Non-GPS")
    end

    return update, UPDATE_MS
end

return update, UPDATE_MS