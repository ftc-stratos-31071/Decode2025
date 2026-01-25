package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TurretConstants {

    public static double kP = 0.012;
    public static double kI = 0.0;
    public static double kD = 0.0008;

    public static double MAX_POWER = 0.5;

    public static double MIN_DEG = -180.0;
    public static double MAX_DEG = 180.0;

    // Encoder zero offsets (calibrate on robot)
    public static double LEFT_OFFSET_DEG = 0.0;
    public static double RIGHT_OFFSET_DEG = 0.0;
}

