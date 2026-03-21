-- ekf_src_auto_gps.lua
--
-- Automatically switches EKF3 source sets based on GPS reliability.
--
-- Assumptions:
--   Source Set 1 (index 0) = GPS environment
--   Source Set 2 (index 1) = Non-GPS environment (ExternalNav / WheelEncoders / OF, etc.)
--
-- Recommended EKF setup:
--   EK3_SRC1_* = GPS-based source set
--   EK3_SRC2_* = Non-GPS source set
--   EK3_SRC_OPTIONS = 0
--
-- Optional tuning via SCR_USER parameters:
--   SCR_USER1 = minimum satellites required for GPS to be considered reliable      (default 10)
--   SCR_USER2 = maximum GPS horizontal accuracy in meters                          (default 1.5)
--   SCR_USER3 = maximum GPS speed accuracy in m/s                                  (default 0.5)
--   SCR_USER4 = maximum GPS velocity innovation (0 disables this check)            (default 0)
--   SCR_USER5 = vote count before switching sources                                (default 10)
--
-- Behavior:
--   If GPS is reliable, the script votes toward Source Set 1.
--   If GPS is not reliable, the script votes toward Source Set 2.
--   A vote counter is used to avoid rapid source flapping.
--
-- Update period: 200 ms

---@diagnostic disable: need-check-nil

local GPS_SOURCE_SET     = 0   -- EK3_SRC1_* (primary)
local NONGPS_SOURCE_SET  = 1   -- EK3_SRC2_* (secondary)
local UPDATE_MS          = 200

local DEFAULT_MIN_SATS   = 10
local DEFAULT_MAX_HACC   = 1.5
local DEFAULT_MAX_SACC   = 0.5
local DEFAULT_MAX_INNOV  = 0.0 -- disabled by default
local DEFAULT_VOTE_MAX   = 10  -- 10 * 200ms = 2 seconds before switching

local current_source = -1
local vote_counter = 0
local started = false

local function get_param_or_default(name, default_value, allow_zero)
    local v = param:get(name)
    if v == nil then
        return default_value
    end
    if (not allow_zero) and (v <= 0) then
        return default_value
    end
    return v
end

local function play_source_tune(source)
    if source == GPS_SOURCE_SET then
        notify:play_tune("L8C")      -- one lower tone = GPS source
    elseif source == NONGPS_SOURCE_SET then
        notify:play_tune("L12DD")    -- two medium tones = non-GPS source
    end
end

local function switch_source(new_source, reason)
    if new_source == current_source then
        return
    end

    ahrs:set_posvelyaw_source_set(new_source)
    current_source = new_source

    if new_source == GPS_SOURCE_SET then
        gcs:send_text(6, "EKF source -> SRC1 (GPS): " .. reason)
    elseif new_source == NONGPS_SOURCE_SET then
        gcs:send_text(4, "EKF source -> SRC2 (Non-GPS): " .. reason)
    end

    play_source_tune(new_source)
end

local function gps_is_reliable()
    local inst = gps:primary_sensor()
    if inst == nil then
        return false, "no primary GPS"
    end

    local min_sats = get_param_or_default("SCR_USER1", DEFAULT_MIN_SATS, false)
    local max_hacc = get_param_or_default("SCR_USER2", DEFAULT_MAX_HACC, false)
    local max_sacc = get_param_or_default("SCR_USER3", DEFAULT_MAX_SACC, false)
    local max_innov = get_param_or_default("SCR_USER4", DEFAULT_MAX_INNOV, true)

    local status = gps:status(inst)
    if (status == nil) or (status < GPS.GPS_OK_FIX_3D) then
        return false, "fix below 3D"
    end

    local sats = gps:num_sats(inst)
    if (sats == nil) or (sats < min_sats) then
        return false, string.format("satellites low (%s)", tostring(sats))
    end

    local hacc = gps:horizontal_accuracy(inst)
    if (hacc == nil) or (hacc > max_hacc) then
        return false, string.format("hacc bad (%s)", tostring(hacc))
    end

    local sacc = gps:speed_accuracy(inst)
    if (sacc == nil) or (sacc > max_sacc) then
        return false, string.format("sacc bad (%s)", tostring(sacc))
    end

    -- Optional innovation check. Set SCR_USER4 > 0 to enable it.
    if max_innov > 0 then
        local innov = ahrs:get_vel_innovations_and_variances_for_source(3)
        if (innov == nil) or (innov:z() == 0.0) or (math.abs(innov:z()) > max_innov) then
            return false, string.format("innov bad (%s)", innov and tostring(innov:z()) or "nil")
        end
    end

    return true, string.format("sats=%d hacc=%.2f sacc=%.2f", sats, hacc, sacc)
end

function update()
    local vote_max = math.floor(get_param_or_default("SCR_USER5", DEFAULT_VOTE_MAX, false))
    if vote_max < 1 then
        vote_max = DEFAULT_VOTE_MAX
    end

    local gps_ok, reason = gps_is_reliable()

    -- On first run, switch immediately to a sensible source.
    if not started then
        started = true
        if gps_ok then
            vote_counter = -vote_max
            switch_source(GPS_SOURCE_SET, "startup, reliable GPS")
        else
            vote_counter = vote_max
            switch_source(NONGPS_SOURCE_SET, "startup, GPS unreliable: " .. reason)
        end
        return update, UPDATE_MS
    end

    -- Vote toward GPS or non-GPS to avoid flapping.
    if gps_ok then
        vote_counter = math.max(vote_counter - 1, -vote_max)
    else
        vote_counter = math.min(vote_counter + 1, vote_max)
    end

    if vote_counter <= -vote_max then
        switch_source(GPS_SOURCE_SET, reason)
    elseif vote_counter >= vote_max then
        switch_source(NONGPS_SOURCE_SET, reason)
    end

    return update, UPDATE_MS
end

return update, UPDATE_MS