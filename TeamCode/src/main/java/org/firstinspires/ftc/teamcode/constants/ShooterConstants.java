package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {
    public static double servoPos = 0.0;
    public static double motorPower = 0.8;
    public static double zeroPower = 0.0;
    public static double defaultPos = 1.0;

    // PID constants for velocity control (tunable via FTC Dashboard)
    public static double velocityKp = 0.003;    // Increased from 0.0001
    public static double velocityKi = 0.0;      // Start at 0
    public static double velocityKd = 0.0;      // Start at 0
    public static double velocityKf = 0.005;    // Reasonable starting value (was 0.00012)

    // Target shooter RPM for consistent shooting
    public static double targetRPM = 3000.0; // Adjust this to your desired shooting speed

    // Tolerance for RPM control (±)
    public static double rpmTolerance = 150.0;  // Increased from 50 for easier initial tuning

    // Command timing delays (in seconds)
    public static double reverseDelaySeconds = 0.5;
    public static double shootDelaySeconds = 0.75;
}