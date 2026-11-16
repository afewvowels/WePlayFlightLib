--[[ WePlay Flight Library ------------------------------------------------------

Shared flight code for WePlay airplane mods (Long Haul Logistics, Radar Probe
Drone Launcher, Attack Drones). Pulled out of Long Haul Logistics 1.0.2.

  local Flight = require("__WePlayFlightLib__/flight")

The library keeps no state and registers no events. Each mod keeps its own
flight tables in its own `storage` and calls Flight.step(f) once per tick for
each one. A flight table is plain data (plus render objects), so it saves and
loads like anything else in storage.

Conventions
  * Positions are map positions, tiles. y points south.
  * Headings are radians: 0 = north, clockwise (the same as Factorio's
    orientation × 2π). Direction vectors: { x = sin(phi), y = -cos(phi) }.
  * Bank: -1 = full left bank, 0 = level, +1 = full right bank.

Modes
  "parked"  sits still (on a pad, or wherever it stopped)
  "path"    follows a list of points (runway takeoff, turn-radius path, landing)
  "orbit"   circles a point forever
  "follow"  holds a slot in a formation behind a leader flight
  "cruise"  flies straight ahead at cruise altitude

Flight.step(f) returns nil, or one of these events:
  "landed"       a path that ended on a runway has rolled to a stop (now parked)
  "arrived"      an airborne path ended (now cruising straight ahead)
  "orbiting"     the drone reached its circle and started orbiting
  "leader_lost"  a follower's leader was destroyed (now cruising)

Culling: set f.hidden = true for a drone nobody can see and Flight.step keeps
flying it (and moving its map dot) without updating its render objects. Clear
it and set f.frame = -1 to have the next step redraw it.
------------------------------------------------------------------------------]]

local Flight = {}

local TAU = 2 * math.pi
local atan2 = math.atan2 or math.atan
local sqrt, sin, cos, floor, abs, min, max = math.sqrt, math.sin, math.cos, math.floor, math.abs, math.min, math.max

-- Defaults. Any of these can be overridden per flight through Flight.new's options.
Flight.DEFAULTS = {
  speed = 0.4,            -- cruise speed, tiles per tick (24 tiles/s)
  cruise_altitude = 4,    -- tiles
  climb_distance = 16,    -- tiles to climb to (and descend from) cruise altitude
  altitude_ease = 0.05,   -- per tick, for altitude changes outside of paths
  turn_radius = 20,       -- tightest turn in paths; also the radius that counts as a full bank

  headings = 64,          -- heading frames in the sprite sheets
  banks = 1,              -- bank frames per heading. 1 = fake banking by squeezing the sprite
  bank_flip = false,      -- true if bank frame 0 is the RIGHT bank instead of the left
  split_banks = false,    -- true: one animation per bank, named "<name>-0", "<name>-1", ...
                          -- (Factorio animations top out at 255 frames, so 5 banks x 64
                          -- headings has to be split)
  bank_ease = 0.06,       -- per tick: how quickly it rolls into and out of a bank
  bank_slide = 0.3,       -- tiles the plane slides toward the inside of a turn at full bank
  bank_squeeze = 0.35,    -- squeeze mode only: wings shrink by up to 35% at full bank
  bank_tilt = 0.025,      -- squeeze mode only: turns of sprite rotation at full bank (9 degrees)

  shadow_ratio = 0.5,     -- shadow moves this many tiles east per tile of altitude
  render_layer = "air-object",
  map_interval = 10,      -- ticks between map dot moves

  -- Runway geometry, for paths that take off from or land on a pad.
  runway_back = 1.5,      -- tiles from pad center to the far (tower) end
  runway_front = 2.5,     -- tiles from pad center to the stripe end
  climb_out = 15,         -- straight climb-out past the runway before turning
  approach = 15,          -- straight final approach before touchdown

  follow_gain = 0.05,     -- follow mode: how hard a drone steers toward its slot
  follow_max = 2,         -- follow mode: top speed while catching up, × cruise speed
  follow_turn_radius = 6, -- follow mode: tightest turn, tiles, so wingmen bank round instead of snapping
}

-- Vector helpers -------------------------------------------------------------

local function vec(phi) return { x = sin(phi), y = -cos(phi) } end
local function phi_of(d) return atan2(d.x, -d.y) end
local function left_of(d) return { x = d.y, y = -d.x } end
local function right_of(d) return { x = -d.y, y = d.x } end
local function offset(p, d, k) return { x = p.x + d.x * k, y = p.y + d.y * k } end
local function dist(a, b) local dx, dy = b.x - a.x, b.y - a.y return sqrt(dx * dx + dy * dy) end
local function wrap(a) return (a + math.pi) % TAU - math.pi end -- to -pi..pi

local function smoothstep(t)
  if t <= 0 then return 0 end
  if t >= 1 then return 1 end
  return t * t * (3 - 2 * t)
end

Flight.vec, Flight.phi_of = vec, phi_of

-- Factorio direction (defines.direction, 16-way) to a unit vector.
function Flight.dir_vector(direction)
  return vec(direction / 16 * TAU)
end

-- Turn-radius paths ("Dubins paths") -------------------------------------------
-- Arc, straight, arc, where both arcs have radius R. Picks the shortest of the
-- four left/right combinations. Appends points (about every 2 tiles along the
-- arcs) to `points`, ending exactly at p1 facing d1. Returns the path length.

local function side_normal(d, side) return side == "L" and left_of(d) or right_of(d) end
local function turn_delta(from_phi, to_phi, side)
  if side == "R" then return (to_phi - from_phi) % TAU end
  return (from_phi - to_phi) % TAU
end

-- `first` ("L" or "R", optional) only allows paths that start by turning that way,
-- and `last` only allows paths that end by turning that way.
local function best_dubins(p0, d0, p1, d1, R, first, last)
  local best
  for _, a_side in pairs(first and { first } or { "L", "R" }) do
    for _, b_side in pairs(last and { last } or { "L", "R" }) do
      local c0 = offset(p0, side_normal(d0, a_side), R)
      local c1 = offset(p1, side_normal(d1, b_side), R)
      local wx, wy = c1.x - c0.x, c1.y - c0.y
      local d = sqrt(wx * wx + wy * wy)
      local straight, s
      if a_side == b_side then
        if d < 1e-6 then s, straight = d0, 0 else s, straight = { x = wx / d, y = wy / d }, d end
      elseif d >= 2 * R then
        straight = sqrt(d * d - 4 * R * R)
        local w = { x = wx / d, y = wy / d }
        local angle = atan2(2 * R, straight) * (a_side == "L" and 1 or -1)
        local n = left_of(w)
        s = { x = w.x * cos(angle) + n.x * sin(angle), y = w.y * cos(angle) + n.y * sin(angle) }
      end
      if s then
        local phi0, phis, phi1 = phi_of(d0), phi_of(s), phi_of(d1)
        local turn0, turn1 = turn_delta(phi0, phis, a_side), turn_delta(phis, phi1, b_side)
        local length = R * turn0 + straight + R * turn1
        if not best or length < best.length then
          best = { length = length, a = a_side, b = b_side, c0 = c0, c1 = c1, s = s,
                   phi0 = phi0, phis = phis, turn0 = turn0, turn1 = turn1 }
        end
      end
    end
  end
  return best
end

local function add_dubins(points, p0, d0, p1, d1, R, first, last)
  -- Constrained turns when asked, falling back to fewer constraints if that
  -- combination can't reach (opposite-turn paths need the circles 2R apart).
  local best = (first or last) and best_dubins(p0, d0, p1, d1, R, first, last)
    or (first and best_dubins(p0, d0, p1, d1, R, first))
    or best_dubins(p0, d0, p1, d1, R)
  local function arc(center, side, start_phi, delta)
    -- A point every 2 tiles, and at least every 0.08 rad (under one of 64
    -- headings), so tight arcs turn smoothly instead of in visible steps.
    local steps = max(1, math.ceil(R * delta / 2), math.ceil(delta / 0.08))
    local k = side == "L" and 1 or -1
    for i = 1, steps do
      local phi = start_phi + (side == "R" and 1 or -1) * delta * i / steps
      points[#points + 1] = { x = center.x + k * R * cos(phi), y = center.y + k * R * sin(phi) }
    end
  end
  arc(best.c0, best.a, best.phi0, best.turn0)
  points[#points + 1] = offset(best.c1, side_normal(best.s, best.b), -R) -- start of the second arc
  arc(best.c1, best.b, best.phis, best.turn1)
  points[#points] = { x = p1.x, y = p1.y } -- land exactly on the end point
  return best.length
end

Flight.add_dubins = add_dubins

-- Creating and destroying ---------------------------------------------------------

--[[ Flight.new(options) -> flight
  Required:
    surface   LuaSurface
    position  where the drone is
    sprites   { body = "animation-name", shadow = "...", glow = "..." }  (shadow, glow optional)
  Optional:
    heading   radians (0 = north, clockwise), or direction = defines.direction
    altitude  starting altitude in tiles (default 0)
    glow_tint color for the glow layer (default white)
    map       { sprite = "item/x", tint = color, scale = 3, visible = true } for a map dot
    any key from Flight.DEFAULTS
]]
function Flight.new(o)
  local f = { mode = "parked" }
  for k, v in pairs(Flight.DEFAULTS) do f[k] = v end
  for k, v in pairs(o) do f[k] = v end
  f.sprites = nil
  f.map = nil

  if o.direction and not o.heading then f.heading = o.direction / 16 * TAU end
  f.heading = f.heading or 0
  f.altitude = f.altitude or 0
  f.pos = { x = o.position.x, y = o.position.y }
  f.position = nil
  f.vx, f.vy = 0, 0
  f.bank, f.bank_target = 0, 0
  f.avg_turn, f.avg_move = 0, 0
  f.age = 0
  f.frame = -1

  local s = o.sprites
  f.anim = { r_body = s.body, r_shadow = s.shadow, r_glow = s.glow }
  f.bank_index = floor((f.banks - 1) / 2 + 0.5) -- level
  local function draw(name, tint)
    if f.split_banks then name = name .. "-" .. f.bank_index end
    return rendering.draw_animation({
      animation = name,
      animation_speed = 0,
      animation_offset = 0,
      target = f.pos,
      surface = f.surface,
      render_layer = f.render_layer,
      tint = tint,
    })
  end
  -- Shadow first so the plane always draws on top of it.
  if s.shadow then f.r_shadow = draw(s.shadow) end
  f.r_body = draw(s.body)
  if s.glow then f.r_glow = draw(s.glow, o.glow_tint) end
  if o.map then
    f.r_map = rendering.draw_sprite({
      sprite = o.map.sprite,
      target = f.pos,
      surface = f.surface,
      tint = o.map.tint,
      x_scale = o.map.scale or 3,
      y_scale = o.map.scale or 3,
      render_mode = "chart",
      visible = o.map.visible ~= false,
    })
    f.map_visible = o.map.visible ~= false
  end
  Flight.render(f)
  return f
end

function Flight.destroy(f)
  for _, key in pairs({ "r_body", "r_shadow", "r_glow", "r_map" }) do
    local r = f[key]
    if r and r.valid then r.destroy() end
    f[key] = nil
  end
  f.dead = true
end

-- Where the drone is: ground position, altitude, and the point in the air where
-- it's drawn (handy for laser beams and explosions).
function Flight.ground(f) return { x = f.pos.x, y = f.pos.y } end
function Flight.air(f) return { x = f.pos.x, y = f.pos.y - f.altitude } end

function Flight.set_map_visible(f, visible)
  f.map_visible = visible
  if f.r_map and f.r_map.valid then
    f.r_map.visible = visible
    if visible then f.r_map.target = { x = f.pos.x, y = f.pos.y } end
  end
end

function Flight.set_glow_tint(f, tint)
  if f.r_glow and f.r_glow.valid then f.r_glow.color = tint end
end

-- Path mode ------------------------------------------------------------------------

local function start_path(f, points, roll_up, roll_down, alt_end, after)
  local cumulative = { 0 }
  for i = 2, #points do cumulative[i] = cumulative[i - 1] + dist(points[i - 1], points[i]) end
  f.mode = "path"
  f.path = {
    points = points,
    cumulative = cumulative,
    total = max(0.001, cumulative[#points]),
    roll_up = roll_up,
    roll_down = roll_down,
    alt_start = f.altitude,
    alt_end = alt_end,
    s = 0,
    segment = 1,
    after = after,
  }
end

-- Runway takeoff points from a pad: tower end, lift-off point, end of climb-out.
local function takeoff_points(f, points, pad, pad_dir)
  points[#points + 1] = offset(pad, pad_dir, -f.runway_back)
  points[#points + 1] = offset(pad, pad_dir, f.runway_front)
  points[#points + 1] = offset(points[#points], pad_dir, f.climb_out)
  return points[#points]
end

-- Fly from wherever the drone is (or take off from a pad) to land on a pad.
--   opts.from_pad, opts.from_dir  take off from this pad position, facing this
--                                 direction vector (omit when already airborne)
--   to_pad, to_dir                the destination pad and the direction it faces;
--                                 drones touch down at the stripe end, rolling toward the tower
--   opts.straight = true          skip the fancy path: straight line pad to pad
--   opts.first_turn, opts.last_turn = "L" / "R"
--                                 make the path start / end with a turn that way.
--                                 first_turn = last_turn = "R" (or both "L") puts
--                                 departures and arrivals on opposite circles around
--                                 each pad, like an airport traffic pattern
function Flight.fly_to_pad(f, to_pad, to_dir, opts)
  opts = opts or {}
  local points, roll_up = {}, 0
  if opts.straight then
    points[1] = { x = (opts.from_pad or f.pos).x, y = (opts.from_pad or f.pos).y }
    points[2] = { x = to_pad.x, y = to_pad.y }
    start_path(f, points, 0, 0, 0)
    return
  end
  local p0, d0
  if opts.from_pad then
    p0 = takeoff_points(f, points, opts.from_pad, opts.from_dir)
    d0 = opts.from_dir
    roll_up = dist(points[1], points[2])
  else
    points[1] = { x = f.pos.x, y = f.pos.y }
    p0, d0 = points[1], vec(f.heading)
  end
  local touchdown = offset(to_pad, to_dir, f.runway_front)
  local final = offset(touchdown, to_dir, f.approach)
  add_dubins(points, p0, d0, final, { x = -to_dir.x, y = -to_dir.y }, f.turn_radius, opts.first_turn, opts.last_turn)
  points[#points + 1] = touchdown
  points[#points + 1] = offset(to_pad, to_dir, -f.runway_back) -- roll to a stop by the tower
  start_path(f, points, roll_up, dist(touchdown, points[#points]), 0)
end

-- Fly (airborne, or taking off from a pad) to a point, arriving facing `end_dir`
-- (a direction vector). Without end_dir it arrives facing straight away from
-- where it started. Ends in "cruise" mode with an "arrived" event.
function Flight.fly_to(f, target, end_dir, opts)
  opts = opts or {}
  local points, roll_up = {}, 0
  local p0, d0
  if opts.from_pad then
    p0 = takeoff_points(f, points, opts.from_pad, opts.from_dir)
    d0 = opts.from_dir
    roll_up = dist(points[1], points[2])
  else
    points[1] = { x = f.pos.x, y = f.pos.y }
    p0, d0 = points[1], vec(f.heading)
  end
  if not end_dir then
    local d = dist(p0, target)
    end_dir = d > 0.001 and { x = (target.x - p0.x) / d, y = (target.y - p0.y) / d } or d0
  end
  add_dubins(points, p0, d0, target, end_dir, f.turn_radius, opts.first_turn)
  start_path(f, points, roll_up, 0, f.cruise_altitude, opts.after)
end

-- Circle `center` at `radius` tiles, clockwise unless clockwise == false.
-- Flies a turn-radius path to the best entry point on the circle first (taking
-- off from a pad if opts.from_pad is given), then orbits forever.
--   opts.first_turn = "L" / "R": the path has to start with a turn that way
--   opts.entry_spread = 0.5: pick at random among entry points whose path is at
--     most 50% longer than the best one, so a group doesn't all take the same line
function Flight.orbit(f, center, radius, clockwise, opts)
  opts = opts or {}
  local spin = clockwise == false and -1 or 1
  local orbit = { center = { x = center.x, y = center.y }, radius = radius, spin = spin }

  if f.mode == "orbit" and not opts.from_pad and dist(f.orbit.center, center) < 1 and f.orbit.spin == spin then
    -- Already circling this spot: just change the radius (it drifts over smoothly).
    f.orbit.target_radius = radius
    return
  end
  local start = opts.from_pad and offset(offset(opts.from_pad, opts.from_dir, f.runway_front), opts.from_dir, f.climb_out) or f.pos
  local start_dir = opts.from_pad and opts.from_dir or vec(f.heading)

  -- Try entry points every 30 degrees around the circle; keep the shortest path
  -- (or, with entry_spread, a random one that's nearly as short).
  local first = opts.first_turn
  local options, best_len = {}, nil
  for i = 0, 11 do
    local a = i / 12 * TAU
    local entry = offset(center, vec(a), radius)
    local b = best_dubins(start, start_dir, entry, vec(a + spin * math.pi / 2), f.turn_radius, first)
    if b then
      options[#options + 1] = { a = a, len = b.length }
      if not best_len or b.length < best_len then best_len = b.length end
    end
  end
  if not best_len and first then -- that turn can't get there: allow either
    first = nil
    return Flight.orbit(f, center, radius, clockwise,
      { from_pad = opts.from_pad, from_dir = opts.from_dir, entry_spread = opts.entry_spread })
  end
  local pool = {}
  for _, o in pairs(options) do
    if o.len <= best_len * (1 + (opts.entry_spread or 0)) + 1e-6 then pool[#pool + 1] = o end
  end
  local pick = opts.entry_spread and pool[math.random(#pool)] or pool[1]
  for _, o in pairs(pool) do -- without spread: the shortest
    if not opts.entry_spread and o.len < pick.len then pick = o end
  end
  local entry = offset(center, vec(pick.a), radius)
  Flight.fly_to(f, entry, vec(pick.a + spin * math.pi / 2),
    { from_pad = opts.from_pad, from_dir = opts.from_dir, after = orbit, first_turn = first })
end

-- Follow a leader flight, holding `slot` = { forward = tiles, right = tiles }
-- relative to the leader's position and heading. Followers should be stepped
-- after their leader each tick. See Flight.formation_slot for ready-made slots.
function Flight.follow(f, leader, slot)
  f.mode = "follow"
  f.leader = leader
  f.slot = { forward = slot.forward, right = slot.right }
  f.slot_last = nil
  f.path = nil
end

-- A "V" formation: slot 1 is behind-left of the leader, 2 behind-right, 3 further
-- behind-left, and so on. `spacing` is in tiles (default 2.5).
function Flight.formation_slot(index, spacing)
  spacing = spacing or 2.5
  local rank = math.ceil(index / 2)
  local side = index % 2 == 1 and -1 or 1
  return { forward = -rank * spacing, right = side * rank * spacing }
end

-- Fly straight ahead at cruise altitude.
function Flight.cruise(f)
  f.mode = "cruise"
  f.path, f.leader = nil, nil
end

-- Stop dead where it is (on the ground; for parking without a path).
function Flight.park(f)
  f.mode = "parked"
  f.path, f.leader, f.orbit = nil, nil, nil
  f.vx, f.vy = 0, 0
end

-- Speed and altitude profiles ---------------------------------------------------

-- Speed ramps up on the takeoff roll and down on the approach and landing roll.
-- The blends toward cruise speed are capped at 1, so a cruise speed slower than
-- the runway speeds (under ~15 tiles/s) just flies at cruise speed instead of
-- overshooting below zero.
local function path_speed(f, p)
  local s, top = p.s, f.speed
  local up
  if p.roll_up > 0 and s < p.roll_up then
    up = 0.05 + 0.2 * (s / p.roll_up)
  elseif p.roll_up > 0 then
    up = 0.25 + (top - 0.25) * min(1, (s - p.roll_up) / 10)
  else
    up = top
  end
  local down = top
  if p.roll_down > 0 then
    local r = p.total - s
    if r < p.roll_down then
      down = 0.04 + 0.16 * (r / p.roll_down)
    else
      down = 0.2 + (top - 0.2) * min(1, (r - p.roll_down) / 12)
    end
  end
  return max(0.03, min(up, down, top))
end

-- Climbs from the starting altitude to cruise, and descends to the end altitude.
local function path_altitude(f, p)
  local air = p.total - p.roll_up - p.roll_down
  if air <= 0 then return p.alt_end end
  local climb = min(f.climb_distance, air / 2)
  local up = smoothstep((p.s - p.roll_up) / climb)
  local down = smoothstep((p.total - p.roll_down - p.s) / climb)
  local cruise = f.cruise_altitude
  return min(p.alt_start + (cruise - p.alt_start) * up, p.alt_end + (cruise - p.alt_end) * down)
end

local function path_position(p)
  local points, cumulative = p.points, p.cumulative
  local i = p.segment
  while i < #points - 1 and cumulative[i + 1] <= p.s do i = i + 1 end
  p.segment = i
  local a, b = points[i], points[i + 1]
  local length = cumulative[i + 1] - cumulative[i]
  local t = length > 0 and (p.s - cumulative[i]) / length or 1
  return { x = a.x + (b.x - a.x) * t, y = a.y + (b.y - a.y) * t }, a, b
end

-- Stepping -------------------------------------------------------------------------

local function ease_altitude(f, target)
  f.altitude = f.altitude + (target - f.altitude) * f.altitude_ease
end

-- Move one tick. Returns the new position, the heading to face, and an event.
local function move(f)
  local mode = f.mode

  if mode == "path" then
    local p = f.path
    p.s = p.s + path_speed(f, p)
    if p.s >= p.total then
      local last, prev = p.points[#p.points], p.points[#p.points - 1]
      local phi = prev and phi_of({ x = last.x - prev.x, y = last.y - prev.y }) or f.heading
      f.altitude = p.alt_end
      f.path = nil
      if p.after then
        f.mode = "orbit"
        f.orbit = p.after
        f.orbit.angle = phi_of({ x = last.x - f.orbit.center.x, y = last.y - f.orbit.center.y })
        return { x = last.x, y = last.y }, phi, "orbiting"
      elseif p.alt_end <= 0 then
        f.mode = "parked"
        return { x = last.x, y = last.y }, phi, "landed"
      else
        f.mode = "cruise"
        return { x = last.x, y = last.y }, phi, "arrived"
      end
    end
    local pos, a, b = path_position(p)
    f.altitude = path_altitude(f, p)
    return pos, phi_of({ x = b.x - a.x, y = b.y - a.y })

  elseif mode == "orbit" then
    local o = f.orbit
    if o.target_radius then
      o.radius = o.radius + (o.target_radius - o.radius) * 0.02
      if abs(o.target_radius - o.radius) < 0.05 then o.radius, o.target_radius = o.target_radius, nil end
    end
    o.angle = (o.angle + o.spin * f.speed / o.radius) % TAU
    ease_altitude(f, f.cruise_altitude)
    return offset(o.center, vec(o.angle), o.radius), o.angle + o.spin * math.pi / 2

  elseif mode == "follow" then
    local leader = f.leader
    if not leader or leader.dead then
      f.mode, f.leader = "cruise", nil
      return { x = f.pos.x + f.vx, y = f.pos.y + f.vy }, f.heading, "leader_lost"
    end
    local fwd = vec(leader.heading)
    local right = right_of(fwd)
    local tx = leader.pos.x + fwd.x * f.slot.forward + right.x * f.slot.right
    local ty = leader.pos.y + fwd.y * f.slot.forward + right.y * f.slot.right
    -- Match the slot's own motion (on the outside of a turn a slot moves faster
    -- than the leader), plus a pull toward the slot to close any gap.
    local last = f.slot_last or { x = tx, y = ty }
    f.slot_last = { x = tx, y = ty }
    local vx = (tx - last.x) + (tx - f.pos.x) * f.follow_gain
    local vy = (ty - last.y) + (ty - f.pos.y) * f.follow_gain
    -- Never turn back: if the slot is behind (the drone got ahead of the
    -- formation), ease off and let the formation catch up instead of looping.
    local along = vx * fwd.x + vy * fwd.y
    local min_along = f.speed * 0.5
    if along < min_along then
      vx, vy = vx + fwd.x * (min_along - along), vy + fwd.y * (min_along - along)
    end
    -- Steer toward that like a plane: keep moving, and turn no tighter than
    -- follow_turn_radius, so a wingman far out of position swings round in an
    -- arc instead of snapping to face its slot.
    local speed = sqrt(vx * vx + vy * vy)
    speed = max(f.speed * 0.5, min(speed, f.speed * f.follow_max))
    local want = (vx * vx + vy * vy) > 1e-6 and phi_of({ x = vx, y = vy }) or f.heading
    local max_turn = speed / f.follow_turn_radius
    local heading = f.heading + max(-max_turn, min(max_turn, wrap(want - f.heading)))
    ease_altitude(f, leader.altitude)
    local d = vec(heading)
    return { x = f.pos.x + d.x * speed, y = f.pos.y + d.y * speed }, heading

  elseif mode == "cruise" then
    ease_altitude(f, f.cruise_altitude)
    return offset(f.pos, vec(f.heading), f.speed), f.heading
  end

  return f.pos, f.heading -- parked
end

-- Move the drone one tick and redraw it. Call once per tick per flight (leaders
-- before their followers). Returns an event string or nil (see the top of the file).
function Flight.step(f)
  if f.dead then return end
  f.age = f.age + 1
  local pos, heading, event = move(f)

  -- Velocity, and how sharply it's turning (averaged over a few ticks so the
  -- corners between path points don't flicker the bank).
  local vx, vy = pos.x - f.pos.x, pos.y - f.pos.y
  local moved = sqrt(vx * vx + vy * vy)
  local turn = moved > 1e-4 and wrap(heading - f.heading) or 0
  f.avg_turn = f.avg_turn + (turn - f.avg_turn) * 0.15
  f.avg_move = f.avg_move + (moved - f.avg_move) * 0.15
  local curvature = f.avg_move > 1e-3 and f.avg_turn / f.avg_move or 0
  f.bank_target = max(-1, min(1, curvature * f.turn_radius))

  f.vx, f.vy = vx, vy
  f.pos = pos
  if moved > 1e-4 or f.mode == "parked" then f.heading = heading % TAU end

  -- Ease into the bank, and stay level near the ground (takeoff roll, touchdown).
  local level = f.bank_target * min(1, f.altitude / max(0.001, f.cruise_altitude))
  if not f.banking_off then f.bank = f.bank + (level - f.bank) * f.bank_ease else f.bank = 0 end

  Flight.render(f)
  return event
end

-- Drawing ----------------------------------------------------------------------------

-- Hysteresis, in frames: a frame only changes once the true value is this far
-- past the halfway point to the next one, so it doesn't flicker back and forth
-- when it hovers right on a boundary (tight turns, formation flying).
local STICKY = 0.2

local AIR_KEYS = { "r_body", "r_glow" }

function Flight.render(f)
  local headings = f.headings
  local exact = f.heading / TAU * headings
  local heading_frame = f.heading_frame
  if heading_frame then
    local diff = (exact - heading_frame + headings / 2) % headings - headings / 2
    if abs(diff) > 0.5 + STICKY then heading_frame = nil end
  end
  heading_frame = heading_frame or floor(exact + 0.5) % headings
  f.heading_frame = heading_frame
  local bank = f.bank
  local frame = heading_frame
  local sx, sy, tilt = 1, 1, 0

  local wing = vec(f.heading + math.pi / 2) -- right wing direction
  local ox, oy = wing.x * f.bank_slide * bank, wing.y * f.bank_slide * bank

  if f.banks > 1 then
    -- Real bank frames: frame = bank_index * headings + heading, or with
    -- split_banks, animation "<name>-<bank_index>" at frame = heading.
    local exact_bank = (bank + 1) / 2 * (f.banks - 1)
    if f.bank_flip then exact_bank = f.banks - 1 - exact_bank end
    local index = f.bank_index
    if not index or abs(exact_bank - index) > 0.5 + STICKY then index = floor(exact_bank + 0.5) end
    if f.split_banks then
      if index ~= f.bank_index then
        f.bank_index = index
        for key, name in pairs(f.anim) do
          local r = f[key]
          if r and r.valid then r.animation = name .. "-" .. index end
        end
      end
    else
      f.bank_index = index
      frame = index * headings + heading_frame
    end
  elseif bank > 0.005 or bank < -0.005 then
    -- Fake it: squeeze the sprite along the wing axis (screen-axis scales, so
    -- diagonal headings squeeze a blend of both) and tilt it a little.
    local phi = heading_frame / headings * TAU
    local wx, wy = cos(phi), sin(phi)
    local squeeze = f.bank_squeeze * abs(bank)
    sx, sy = 1 - squeeze * wx * wx, 1 - squeeze * wy * wy
    tilt = f.bank_tilt * bank * wx
  end

  -- The map dot, only while it's shown (and only every map_interval ticks).
  if f.map_visible and f.age % f.map_interval == 0 then
    local r_map = f.r_map
    if r_map and r_map.valid then r_map.target = { x = f.pos.x, y = f.pos.y } end
  end

  -- f.hidden: the mod is culling this drone (nobody near enough to see it). Keep
  -- the heading and bank state up to date, but don't touch the render objects.
  if f.hidden then return end

  local x, y, alt = f.pos.x + ox, f.pos.y + oy, f.altitude
  local air = { x = x, y = y - alt }
  local changed = frame ~= f.frame
  f.frame = frame

  -- Every render-object read or write is an API call, so only touch what changed:
  -- positions every tick, frame / scale / tilt only when they differ from what's
  -- drawn, and validity only on redraws and once a second.
  local full = changed or f.age % 60 == 0
  local look = full or sx ~= f.drawn_sx or sy ~= f.drawn_sy or tilt ~= f.drawn_tilt
  f.drawn_sx, f.drawn_sy, f.drawn_tilt = sx, sy, tilt

  for _, key in pairs(AIR_KEYS) do
    local r = f[key]
    if r and (not full or r.valid) then
      r.target = air
      if changed then r.animation_offset = frame end
      if look then
        r.x_scale, r.y_scale = sx, sy
        r.orientation = tilt % 1
      end
    elseif r then
      f[key] = nil
    end
  end
  local shadow = f.r_shadow
  if shadow and (not full or shadow.valid) then
    shadow.target = { x = x + alt * f.shadow_ratio, y = y }
    if changed then shadow.animation_offset = frame end
    if look then shadow.x_scale, shadow.y_scale = sx, sy end
  elseif shadow then
    f.r_shadow = nil
  end
end

-- Sounds -------------------------------------------------------------------------------

-- Play a flying sound for drones near players, at most `max_sounds` at a time so a
-- swarm doesn't get deafening. Call every ~0.5 s with a table of flights.
function Flight.play_sounds(flights, sound, range, max_sounds)
  range, max_sounds = range or 48, max_sounds or 6
  local played = 0
  for _, player in pairs(game.connected_players) do
    for _, f in pairs(flights) do
      if played >= max_sounds then return end
      if not f.dead and f.surface.valid and f.surface == player.surface then
        local p = Flight.air(f)
        local dx, dy = p.x - player.position.x, p.y - player.position.y
        if dx * dx + dy * dy < range * range then
          f.surface.play_sound({ path = sound, position = p })
          played = played + 1
        end
      end
    end
  end
end

return Flight
