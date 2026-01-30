package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {
    // Servo positions
    public static double servoPos = 0.0;
    public static double defaultPos = 1.0;

    public static double zeroPower = 0.0;
    public static double kF = 0.000195;
    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.0;

    // PIDF output limits
    public static double MIN_POWER = 0.0;
    public static double MAX_POWER = 1.0;

    // Default target RPM for PIDF control
    public static double closeTargetRPM = 4500.0;
    public static double farTargetRPM = 3450.0;
    public static double tolRpm = 50.0;
    public static double tolRpm2 = 100.0;
    public static double TICKS_PER_REV = 28.0;
}