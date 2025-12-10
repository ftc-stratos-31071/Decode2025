package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class LimelightConstants {
    // Measure / estimate these on your robot + field and tune in FTC Dashboard

    // Height of camera lens above the floor (meters)
    public static double CAM_HEIGHT_M = 0.30;

    // Height of the AprilTag you’re shooting at (center of tag, meters)
    public static double TAG_HEIGHT_M = 1.20;

    // Camera pitch relative to the floor, in degrees (upwards positive)
    public static double CAM_PITCH_DEG = 20.0;

    // Clamp distance to a sane range
    public static double MIN_DISTANCE_M = 0.5;
    public static double MAX_DISTANCE_M = 5.0;

    // Optional scale to tweak all distances if they’re consistently off
    public static double DISTANCE_SCALE = 1.0;
}