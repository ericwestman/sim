FILE(REMOVE_RECURSE
  "src/sim/msg"
  "src/sim/srv"
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "msg_gen/cpp/include/sim/Command.h"
  "msg_gen/cpp/include/sim/TelemetryUpdate.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
