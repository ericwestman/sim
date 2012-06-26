FILE(REMOVE_RECURSE
  "src/sim/msg"
  "src/sim/srv"
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "src/sim/srv/__init__.py"
  "src/sim/srv/_SaveFlightData.py"
  "src/sim/srv/_RequestWaypointInfo.py"
  "src/sim/srv/_CreateSimulatedPlane.py"
  "src/sim/srv/_GoToWaypoint.py"
  "src/sim/srv/_DeleteSimulatedPlane.py"
  "src/sim/srv/_RequestPlaneID.py"
  "src/sim/srv/_LoadPath.py"
  "src/sim/srv/_LoadCourse.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
