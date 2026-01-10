package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {
    // Servo positions
    public static double servoPos = 0.6;
    public static double defaultPos = 0.2;
    public static double kickDefaultPos = 0.0;
    public static double kickerPos = 0.425;

    // Motor power (for open-loop control)
    public static double motorPower = 0.8;
    public static double zeroPower = 0.0;

    // Command timing delays (in seconds)
    public static double reverseDelaySeconds = 0.5;
    public static double shootDelaySeconds = 0.75;

    // ===== PIDF GAINS - TUNABLE VIA FTC DASHBOARD =====
    // See tuning guide above for how to adjust these values
    public static double kF = 0.00017;
    public static double kP = 0.00008;

    //not implemented in error either so if you do this implement there as well
    public static double kI = 0.0;
    public static double kD = 0.0;

    // PIDF output limits
    public static double MIN_POWER = 0.0;
    public static double MAX_POWER = 1.0;

    // Default target RPM for PIDF control
    public static double defaultTargetRPM = 3500.0;
}