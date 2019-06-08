wait until ship:loaded and ship:unpacked.
core:doevent("open terminal").
wait 0.
if homeconnection:isconnected {
  copypath("0:/"+ship:name+".ks", "1:/mission.ks").
  print "Updated mission script!".
}
runpath("1:/mission.ks").