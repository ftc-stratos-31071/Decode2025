package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {
    // Servo positions
    public static double servoPos = 0.4;
    public static double defaultPos = 1.0;
    public static double farHoodPos = 0.1;
    public static double zeroPower = 0.0;
    public static double kF = 0.000172;
    public static double kP = 0.0019;
    public static double kI = 0.0;
    public static double kD = 0.0;

    // PIDF output limits
    public static double MIN_POWER = 0.0;
    public static double MAX_POWER = 1.0;

    // Default target RPM for PIDF control
    public static double closeTargetRPM = 3700.0;
    public static double farTargetRPM = 5000.0;
    public static double tolRpm = 50.0;
    public static double tolRpm2 = 100.0;
    public static double TICKS_PER_REV = 28.0;

    // ========== BATTERY COMPENSATION ==========
    // IMPORTANT: Set NOMINAL_VOLTAGE to your battery voltage when you tuned PIDF!
    // How to find it:
    //   1. Check Driver Station voltage display (top bar)
    //   2. Run FTC Dashboard and look at "Battery Voltage" telemetry
    //   3. Use a voltmeter on your battery
    //
    // Example values:
    //   - Fresh battery: 13.0-13.5V
    //   - Partially used: 12.0-12.8V
    //   - Low battery: 11.0-11.5V
    //
    // If you're unsure, start with 12.5V (typical practice battery)
    public static double NOMINAL_VOLTAGE = 13;  // TODO: Set to actual voltage when you tuned PIDF!

    // Toggle compensation on/off (usually leave at true)
    public static boolean ENABLE_BATTERY_COMPENSATION = true;
}