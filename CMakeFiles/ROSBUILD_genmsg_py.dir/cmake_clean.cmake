FILE(REMOVE_RECURSE
  "src/sim/msg"
  "src/sim/srv"
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/sim/msg/__init__.py"
  "src/sim/msg/_Command.py"
  "src/sim/msg/_TelemetryUpdate.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
