"""Spark schema for frame records written by the C++ JsonlExporter.

Declaring the schema up front (instead of inference) keeps Auto Loader stable when early
files lack optional blocks such as IMU or vehicle fields.
"""

from pyspark.sql.types import (
    ArrayType,
    BooleanType,
    DoubleType,
    LongType,
    StringType,
    StructField,
    StructType,
)

OBSTACLE_SCHEMA = StructType([
    StructField("x", DoubleType()),
    StructField("y", DoubleType()),
    StructField("z", DoubleType()),
    StructField("len", DoubleType()),
    StructField("wid", DoubleType()),
    StructField("hgt", DoubleType()),
    StructField("range_m", DoubleType()),
    StructField("points", LongType()),
])

FRAME_SCHEMA = StructType([
    StructField("vehicle_id", StringType()),
    StructField("run_id", StringType()),
    StructField("seq", LongType()),
    StructField("sensor_ts_ns", LongType()),
    StructField("e2e_latency_ms", DoubleType()),
    StructField("lidar_proc_ms", DoubleType()),
    StructField("over_budget", BooleanType()),
    StructField("raw_points", LongType()),
    StructField("filtered_points", LongType()),
    StructField("ground_points", LongType()),
    StructField("num_obstacles", LongType()),
    StructField("nearest_obstacle_m", DoubleType()),
    StructField("ttc_s", DoubleType()),
    StructField("imu_valid", BooleanType()),
    StructField("vehicle_valid", BooleanType()),
    StructField("roll_rad", DoubleType()),
    StructField("pitch_rad", DoubleType()),
    StructField("yaw_rad", DoubleType()),
    StructField("yaw_rate_rps", DoubleType()),
    StructField("long_accel_mps2", DoubleType()),
    StructField("lat_accel_mps2", DoubleType()),
    StructField("accel_norm_mps2", DoubleType()),
    StructField("speed_mps", DoubleType()),
    StructField("steering_deg", DoubleType()),
    StructField("engine_rpm", DoubleType()),
    StructField("throttle_pct", DoubleType()),
    StructField("brake_bar", DoubleType()),
    StructField("gear", LongType()),
    StructField("can_decode_errors", LongType()),
    StructField("obstacles", ArrayType(OBSTACLE_SCHEMA)),
])
