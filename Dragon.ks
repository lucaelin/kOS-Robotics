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

SAS on.
RCS on.

set TARGET to Vessel("Station").
awaitInput().
local p is target:position.
until not HASTARGET {
  clearvecdraws().
  local diff is target:position - p.
  VECDRAW(V(0,0,0), diff, blue, "diff", 1, true).
  local sdiff is TARGET:VELOCITY:ORBIT - SHIP:VELOCITY:ORBIT.
  set diff to diff + sdiff.

  set ship:control:FORE to VDOT(ship:facing:forevector, diff) * 0.2.
  set ship:control:STARBOARD to VDOT(ship:facing:starvector, diff) * 0.2.
  set ship:control:TOP to VDOT(ship:facing:topvector, diff) * 0.2.

  wait 0.
}

RCS off.

SHUTDOWN.