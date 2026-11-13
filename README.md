# WePlay: Flight Library

Shared flight code for WePlay airplane mods: runway takeoffs and landings, turn-radius paths, circling, formation flying, altitude, shadows and banking. It does nothing on its own.

The flight code comes from **Long Haul Logistics 1.0.2**, with circling, formations and real bank frames added for **Attack Drones**.

## Using it

Add it to your mod's `info.json` dependencies:

```json
"dependencies": ["base >= 2.1", "WePlayFlightLib >= 0.1.0"]
```

Then in `control.lua`:

```lua
local Flight = require("__WePlayFlightLib__/flight")
```

The library has **no events and no saved data of its own**. Your mod keeps its flight tables in its own `storage` and calls `Flight.step(f)` once per tick for each one. A flight table is plain data plus render objects, so it saves and loads like anything else.

## Creating a flight

```lua
local f = Flight.new({
  surface = pad.surface,
  position = pad.position,
  direction = pad.direction,          -- or heading = radians (0 = north, clockwise)
  sprites = {
    body = "wp-attack-drone-hull",
    shadow = "wp-attack-drone-shadow",
    glow = "wp-attack-drone-glow",    -- optional
  },
  glow_tint = { 0.3, 1, 0.4 },        -- optional
  banks = 5,                          -- 5 bank frames per heading (1 = squeeze trick)
  map = { sprite = "item/wp-attack-drone", tint = { 1, 0.2, 0.2 } }, -- optional map dot
})
storage.drones[id] = f
```

Every number in `Flight.DEFAULTS` (speed, cruise altitude, turn radius, banking, runway lengths…) can be overridden the same way.

## Telling it where to go

| Call | What it does | Ends with |
|---|---|---|
| `Flight.fly_to_pad(f, pad_pos, pad_dir, opts)` | Long Haul style: runway takeoff (if `opts.from_pad`), turn-radius path, final approach, landing roll | `"landed"`, parked |
| `Flight.fly_to(f, target, end_dir, opts)` | Turn-radius path to a point, arriving facing `end_dir` | `"arrived"`, then cruising straight |
| `Flight.orbit(f, center, radius, clockwise, opts)` | Flies to the best entry point on the circle, then circles forever | `"orbiting"` |
| `Flight.follow(f, leader, slot)` | Holds a formation slot behind a leader flight | `"leader_lost"` if the leader is destroyed |
| `Flight.cruise(f)` / `Flight.park(f)` | Fly straight ahead / stop where it is | |

`opts.from_pad` and `opts.from_dir` (a direction vector, e.g. `Flight.dir_vector(pad.direction)`) make the drone take off from a pad's runway first.

`Flight.formation_slot(i, spacing)` gives ready-made "V" formation slots: 1 is behind-left, 2 behind-right, 3 further behind-left, and so on.

## Each tick

```lua
script.on_event(defines.events.on_tick, function()
  for id, f in pairs(storage.drones) do
    local event = Flight.step(f)      -- step leaders before their followers
    if event == "landed" then ... end
  end
end)
```

`Flight.air(f)` is where the drone is drawn in the air (use it for laser beams and explosions). `Flight.ground(f)` is the spot under it. `Flight.destroy(f)` removes its sprites.

`Flight.play_sounds(flights, "sound-name", range, max_sounds)` plays a flying sound for drones near players, a few at a time. Call it about every half second.

## Example: Attack Drones

```lua
-- Launch: each drone takes off and joins the squad circling the launchpad.
local f = Flight.new({ ... })
if not squad.leader then
  squad.leader = f
  Flight.orbit(f, loiter_center, 12, true, { from_pad = pad.position, from_dir = Flight.dir_vector(pad.direction) })
else
  squad.count = squad.count + 1
  Flight.fly_to(f, loiter_center, nil, { from_pad = pad.position, from_dir = Flight.dir_vector(pad.direction) })
  -- When it arrives, switch it into the formation:
  -- Flight.follow(f, squad.leader, Flight.formation_slot(squad.count))
end

-- Critical mass reached: the leader heads for the nests and the squad follows.
Flight.orbit(squad.leader, nest_center, 15, false)
```

## Sprites

- **Headings:** frame = heading index, 0 = north, clockwise (`headings = 64` by default).
- **Real bank frames** (`banks > 1`): frame = `bank_index * headings + heading`, where bank index 0 is the full **left** bank and the last one the full **right** bank. If yours are the other way around, set `bank_flip = true`.
- **Too many frames?** Factorio animations max out at 255 frames, so 5 banks × 64 headings won't fit in one. Set `split_banks = true` and make one 64-frame animation per bank, named `<name>-0` … `<name>-4` (`pack_drone_sprites.py --split-banks 5` does this).
- **Squeeze mode** (`banks = 1`): banking is faked by squeezing the sprite along its wings and tilting it slightly, like Long Haul Logistics and Radar Probe Drone Launcher.
- **Shadow:** drawn on the ground, `shadow_ratio` tiles east per tile of altitude. Any offset baked into the shadow render is kept, so a parked drone's shadow sits where it did in Blender.

## Tested

The flight math was tested outside the game (in a Lua 5.3 interpreter with a mocked `rendering` API), not in Factorio itself:

- A pad-to-pad flight lands exactly at the destination runway's tower end.
- A drone taking off from a pad joins a 12-tile circle at exactly 12 tiles and holds a full bank.
- Six followers hold their "V" slots within about half a tile while circling, follow the leader 1,000 tiles to a nest, and switch to cruising when the leader is destroyed.
- Banking rolls in smoothly on turns and levels out on straights.
