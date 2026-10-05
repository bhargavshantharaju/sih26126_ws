# Copyright 2026 DRISTHI Project
# Licensed under the Apache License, Version 2.0
#
# Risk assessment from the risk-aware planner

builtin_interfaces/Time stamp
float32 total_cost         # 0-1 combined risk
float32 terrain_cost       # 0-1 terrain risk component
float32 obstacle_cost      # 0-1 static obstacle risk component
float32 dynamic_risk_cost  # 0-1 dynamic obstacle risk component
float32 localization_risk  # 0-1 localization quality risk
float32 sensor_degradation_cost  # 0-1 sensor health risk
float32 smoothness_cost    # 0-1 path smoothness cost
float32 progress_cost      # 0-1 goal progress rate risk
string risk_level          # LOW, MODERATE, HIGH, CRITICAL
string selected_path_reason  # human explanation of selected path
string[] rejected_alternatives  # reasons why alternatives were rejected
bool path_valid            # false if no valid path exists
