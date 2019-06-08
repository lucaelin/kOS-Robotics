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

function getHinges {
  local ps is List().
  list parts in ps.
  local robotics is List().

  for p in ps {
    for m in p:modules {
      if m:startswith("ModuleRoboticServoHinge") {
        robotics:add(p).
      }
    }
  }
  return robotics.
}
function getServos {
  local ps is List().
  list parts in ps.
  local robotics is List().

  for p in ps {
    for m in p:modules {
      if m:startswith("ModuleRoboticRotationServo") {
        robotics:add(p).
      }
    }
  }
  return robotics.
}

function getJoints {
  parameter hinges is getHinges().
  parameter ref is V(0,0,0).
  local joints is list().

  for p in hinges {
    joints:add(p:position - ref).
  }
  return joints.
}

function range {
  parameter start.
  parameter end.

  local inc is 1.
  if start > end {
    set inc to -1.
  }
  local ret is list().
  FROM {local i is start.} UNTIL i = (end + inc) STEP {set i to i + inc.} DO {
    ret:add(i).
  }
  return ret.
}

function fabrik {
  parameter t is V(0,0,0).
  parameter base is V(0,0,0).
  parameter tip is up:vector.
  parameter p is getJoints().
  parameter tol is 0.001.

  local p is p:copy.
  p:insert(0, base - p[0]).
  p:add(p[p:length-1] + tip).

  local tt is t + tip.

  // number of element
  local n is p:length - 1.

  // distances between joints
  local d is list().
  for i in range(0, n - 1) {
    d:add((p[i+1] - p[i]):mag).
  }
  // length of movable the arm.
  local length is 0.
  for i in range(1, n - 2) {
    set length to length + d[i].
  }

  // the distance between root and target
  local dist is (p[1] - t):mag.
  // check whether the target is within reach
  if dist > length {
    // target is unreachable
    for i in range(1, n - 2) {
      // find the distance r[i] between the target t and the joint position p[i]
      local r is (t - p[i]):mag.
      local l is d[i] / r.
      // find the new joint positions p[i]
      set p[i+1] to (1 - l) * p[i] + l * t.
    }
  } else {
    // target is reachable; thus, set b as the initial position of the joint p1

    local bb is p[0].
    local b is p[1].
    // check whether the distance between the end effector p[n] and the target t is greater than a tolerance
    until (p[n] - tt):mag < tol {
      // stage 1: forward reaching
      // set the end effector p[n] as target t
      set p[n] to tt.
      set p[n-1] to t.
      for i in range(n - 2, 0) {
        // check constrains
        local v1 is p[i] - p[i+1].
        local v2 is p[i+1] - p[i+2].
        if VDOT(v1, v2) < 0 {
          // invalid position
          print "invalid forward @" + i + " with "+VANG(v1, v2).
          set v1 to VXCL(v2, v1):normalized * d[i].
          set p[i] to p[i+1] + v1.
          // vecdraw(p[i+2], p[i+1] - p[i+2], red, i+1, 1, true).
          // vecdraw(p[i+1], p[i] - p[i+1], blue, i, 1, true).
        } else {
          // find the distance r[i] between the new joint position p[i+1] and the joint p[i]
          local r is v1:mag.
          local l is d[i] / r.
          // find the new joint positions p[i]
          set p[i] to (1 - l) * p[i+1] + l * p[i].
        }
      }

      // stage 2: backward reaching
      // set the root p[0] to its initial position
      set p[0] to bb.
      set p[1] to b.
      for i in range(2, n) {
        // check constrains
        local v1 is p[i] - p[i-1].
        local v2 is p[i-1] - p[i-2].
        if VDOT(v1, v2) < 0 {
          // invalid position
          print "invalid backward @" + i + " with "+VANG(v1, v2).
          set v1 to VXCL(v2, v1):normalized * d[i-1].
          set p[i] to p[i-1] + v1.
          // vecdraw(p[i-2], p[i-1] - p[i-2], red, i-1, 1, true).
          // vecdraw(p[i-1], p[i] - p[i-1], blue, i, 1, true).
        } else {
          // find the distance r[i] between the new joint position p[i] and the joint p[i+1]
          local r is v1:mag.
          local l is d[i-1] / r.
          // find the new joint positions p[i]
          set p[i] to (1 - l) * p[i-1] + l * p[i].
        }
      }
      wait 0.
    }
    for i in range(2, n) {
      local v1 is p[i] - p[i-1].
      local v2 is p[i-1] - p[i-2].
      local ang is VANG(v1, v2).
      if ang > 90 print "ERROR! " + i + " " + ang.
    }
  }
  p:remove(0).
  p:remove(p:length-1).
  return p.
}

local targets is list(
  up:vector * 9,
  V(0,3,6),
  V(-2,7,1),
  V(-1,6,3),
  V(0,6,4),
  V(0,0,6),
  V(0,0,6),
  V(0,0,11),
  up:vector * 15
).
local tips is list(
  north:vector,
  north:vector,
  up:vector,
  north:vector,
  north:vector,
  north:vector,
  -north:vector + up:vector,
  up:vector,
  up:vector
).
for x in range(0, targets:length - 1) {
  print "run next target?".
  awaitInput().
  local hinges is getHinges().
  local servos is getServos().
  local root is servos[0]:position.
  local joints is getJoints(hinges, root).
  local t is targets[x] - root.
  local tip is tips[x].
  clearvecdraws().
  //vecdraw(V(0,0,0), t, red, "t", 1, true).
  vecdraw({ return servos[0]:position + t. }, tip, yellow, "tip", 1, true).
  local des is fabrik(t, -up:vector, tip, joints).
  des:add(des[des:length-1] + tip).

  local fore is servos[0]:facing:topvector.
  local top is servos[0]:facing:forevector.
  local star is VCRS(top, -fore).
  for i in range(0, servos:length - 1) {
    //vecdraw(des[i], fore, white, "fore", 1, true).
    //vecdraw(des[i], top, white, "top", 1, true).
    local v is des[i+1] - des[i].

    local h is VANG(top, v).
    //print "hinge"+i+": " + h.
    hinges[i]:getmodule("ModuleRoboticServoHinge"):setField("Target Angle", h).

    set vx to VXCL(top, v).
    local dot is VDOT(vx, star).
    if dot = 0 {
      set dot to 1.
    }
    local sign is dot / abs(dot).
    local s is sign * VANG(vx, -fore).
    //print "servo"+i+": " + s.
    servos[i]:getmodule("ModuleRoboticRotationServo"):setField("Target Angle", s).

    set star to VCRS(top, v):normalized.
    set top to v:normalized.
    set fore to VCRS(top, star):normalized.

    //vecdraw(des[i], des[i+1] - des[i], blue, i, 1, true).
  }
}
print "done.".
