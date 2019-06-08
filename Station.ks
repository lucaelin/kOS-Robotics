function awaitInput {
  until not TERMINAL:INPUT:HASCHAR() {
    TERMINAL:INPUT:GETCHAR().
  }.
  print "Press any key to continute...".
  CORE:DOEVENT("Open Terminal").
  wait until TERMINAL:INPUT:HASCHAR() or not CORE:MESSAGES:EMPTY.
  if TERMINAL:INPUT:HASCHAR() { TERMINAL:INPUT:GETCHAR(). }
  return true.
}

RUNPATH("0:/fabrik.ks").

SAS on.
RCS on.
//if not HASTARGET set TARGET to Vessel("Dragon").
//local grab is target:partstagged("grabDragon")[0].

awaitInput().

until not HASTARGET {
  local t is (grab:position + (1 - SHIP:CONTROL:PILOTMAINTHROTTLE) * 5 * grab:portfacing:vector).
  local tip is -grab:portfacing:vector.
  reach(t, tip).
  wait 1.
}

print "grabbed.".
awaitInput().

local dest is SHIP:partstagged("dest")[0].
local destDragon is SHIP:partstagged("destDragon")[0].
local rotPort is SHIP:partstagged("rotPort")[0].
local grabPort is SHIP:partstagged("grabStation")[0].
local dragonPort is SHIP:partstagged("grabDragon")[0].

until false {
  clearvecdraws().
  local t is dest:position + dest:facing:forevector * (1 - SHIP:CONTROL:PILOTMAINTHROTTLE) * 5 + dest:facing:starvector * 2 + dest:facing:forevector * 0.
  local tip is -dest:facing:starvector.
  local final is reach(t, tip).
  local align is reachSingle(dest:position - t, final["fore"], final["star"], final["top"]).
  
  local sign is VDOT(grabPort:facing:starvector, dragonPort:facing:starvector).
  set sign to sign / ABS(sign).
  local offset is sign * VANG(grabPort:facing:topvector, -dragonPort:facing:topvector) + 90.
  print align["servo"].
  print offset.
  rotPort:getmodule("ModuleRoboticRotationServo"):setField("Target Angle", align["servo"] + offset).
  wait 3.
}