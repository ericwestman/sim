FILE(REMOVE_RECURSE
  "../src/sim/msg"
  "../src/sim/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/sim/SaveFlightData.h"
  "../srv_gen/cpp/include/sim/RequestWaypointInfo.h"
  "../srv_gen/cpp/include/sim/CreateSimulatedPlane.h"
  "../srv_gen/cpp/include/sim/GoToWaypoint.h"
  "../srv_gen/cpp/include/sim/DeleteSimulatedPlane.h"
  "../srv_gen/cpp/include/sim/RequestPlaneID.h"
  "../srv_gen/cpp/include/sim/LoadPath.h"
  "../srv_gen/cpp/include/sim/LoadCourse.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
