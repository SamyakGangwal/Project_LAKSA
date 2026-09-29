"""LAKSA vehicle constants used by the simulator, expert and exported model.

Measured values come from ``laksa_speed_race/SIM_ASSUMPTIONS.json`` and the
production Nav2 footprint.  ``base_footprint`` is assumed to be the rear axle
(front 0.419 m / rear 0.149 m footprint with a 0.324 m wheelbase); the
simulator's single-track state is at the centre of gravity, placed midway
between the axles as in ``laksa_proxy_v0.yaml``.
"""

WHEELBASE_M = 0.324
LF_M = 0.162
LR_M = 0.162
MASS_KG = 3.75
STEER_LEFT_MAX_RAD = 0.523
STEER_RIGHT_MAX_RAD = 0.288

FOOTPRINT_FRONT_M = 0.419   # from base_footprint (rear axle)
FOOTPRINT_REAR_M = 0.149
FOOTPRINT_HALF_WIDTH_M = 0.148
FOOTPRINT_PADDING_M = 0.02

LIDAR_X_FROM_BASE_M = 0.31542
LIDAR_DIST_FROM_COG_M = LIDAR_X_FROM_BASE_M - LR_M

CONTROL_PERIOD_S = 0.08      # ~12.5 Hz, matching the A2M12 scan rate
PHYSICS_DT_S = 0.01

GYM_PARAMS = {
    "mu": 1.0489, "C_Sf": 4.718, "C_Sr": 5.4562,
    "lf": LF_M, "lr": LR_M, "h": 0.074, "m": MASS_KG, "I": 0.04712,
    "s_min": -STEER_RIGHT_MAX_RAD, "s_max": STEER_LEFT_MAX_RAD,
    "sv_min": -3.2, "sv_max": 3.2, "v_switch": 7.319, "a_max": 9.51,
    "v_min": -1.0, "v_max": 6.0,
    "width": 2.0 * FOOTPRINT_HALF_WIDTH_M, "length": FOOTPRINT_FRONT_M + FOOTPRINT_REAR_M,
}
