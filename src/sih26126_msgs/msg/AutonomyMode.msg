# Copyright 2026 DRISTHI Project
# Licensed under the Apache License, Version 2.0
#
# Current autonomy mode of the DRISTHI system

builtin_interfaces/Time stamp
string mode                # FULL_AUTONOMY, DEGRADED_PERCEPTION, DEGRADED_LOCALIZATION,
                           # DEGRADED_SENSOR, RECOVERY, SAFE_HOLD, RETURN_HOME
string reason              # why this mode was entered
bool is_degraded           # true if not FULL_AUTONOMY
float32 capability_score   # 0-1 estimate of current capability
string[] active_degradations  # list of active degradation signals
