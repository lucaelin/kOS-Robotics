function getRobotics {
  local ps is List().
  list parts in ps.
  local robotics is List().

  for p in ps {
    for m in p:modules {
      if m:startswith("ModuleRobotic") {
        robotics:add(p).
      }
    }
  }
  return robotics.
}

local robotics is getRobotics().
local fore is ship:partstagged("F")[0]:getmodule("ModuleRoboticServoRotor").
local left is ship:partstagged("L")[0]:getmodule("ModuleRoboticServoRotor").
local right is ship:partstagged("R")[0]:getmodule("ModuleRoboticServoRotor").
local back is ship:partstagged("B")[0]:getmodule("ModuleRoboticServoRotor").

local kp is 1.
local ki is 0.01.
local kd is 1.

local ud is 0.
local pidud is PIDLOOP(kp, ki, kd).
local lr is 0.
local pidlr is PIDLOOP(kp, ki, kd).
local fb is 0.
local pidfb is PIDLOOP(kp, ki, kd).
local cc is 0.
local pidcc is PIDLOOP(kp, ki, kd).

local targetTop is up:vector.

until false {
  set ud to ship:control:pilotmainthrottle.
  set targetTop to up:vector
    + north:vector * -ship:control:pilotpitch * 0.25
    + VCRS(north:vector, up:vector):normalized * -ship:control:pilotyaw * 0.25.

  set targetHead to north:vector.

  local topOffset is VXCL(ship:facing:forevector, targetTop).
  local headOffset is VXCL(ship:facing:topvector, targethead).

  set efb to -VDOT(topOffset, ship:facing:topvector).
  set elr to VDOT(topOffset, ship:facing:starvector).
  set ecc to VDOT(headOffset, -ship:facing:starvector).

  set fb to pidfb:UPDATE(TIME:SECONDS, efb).
  set lr to pidlr:UPDATE(TIME:SECONDS, elr).
  set cc to pidcc:UPDATE(TIME:SECONDS, ecc).

  print round(fb,5) + " : " + round(lr,5).

  local pf is (-cc) + (-fb) + ud.
  local pb is (-cc) + fb + ud.
  local pl is cc + (-lr) + ud.
  local pr is cc + lr + ud.
  
  fore:setfield("torque limit(%)", ABS(pf) * 100).
  fore:setfield("invert direction", pf > 0).
  back:setfield("torque limit(%)", ABS(pb) * 100).
  back:setfield("invert direction", pb > 0).
  left:setfield("torque limit(%)", ABS(pl) * 100).
  left:setfield("invert direction", pl < 0).
  right:setfield("torque limit(%)", ABS(pr) * 100).
  right:setfield("invert direction", pr < 0).
  
  // fore:setfield("torque limit(%)", pf * 100).
  // back:setfield("torque limit(%)", pb * 100).
  // left:setfield("torque limit(%)", pl * 100).
  // right:setfield("torque limit(%)", pr * 100).
  
  wait 0.
}