# Copyright 2026 DRISTHI Project
# Licensed under the Apache License, Version 2.0
#
# A structured mission intelligence event

builtin_interfaces/Time stamp
string severity            # INFO, WARNING, CRITICAL, RECOVERY, MISSION
string subsystem           # e.g. PLANNER, SAFETY, LOCALIZATION, OBSTACLE, RECOVERY
string description         # Human-readable description
string cause               # What triggered this event
string action_taken        # What the system did in response
string autonomy_mode       # Current autonomy mode at event time
string resolution          # How it was resolved (empty if ongoing)
