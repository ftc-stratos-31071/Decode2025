package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TurretConstants {
    // LOGICAL coordinates (relative to center at 0°)
    public static double DEFAULT_TURRET_DEG = 0.0;   // Center position (faces straight)
    public static double MAX_TURRET_DEG = 180.0;     // 120° right of center
    public static double MIN_TURRET_DEG = -180.0;    // 120° left of center

    // Physical center offset (physical servo center is at 45°)
    // This is used internally by Turret.java to convert logical → physical coordinates
    public static double PHYSICAL_CENTER = 45.0;

    public static double BLUE_FAR_ANGLE = 50;
}