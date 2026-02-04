package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConstants {
    // Servo positions
    public static double servoPos = 0.4;
    public static double defaultPos = 1.0;
    public static double farHoodPos = 0.1;
    public static double zeroPower = 0.0;

    // ========== PIDF TUNING ==========
    // NOTE: Currently using PF control (feedforward + proportional only)
    // This is common for flywheels and works well if kF and kP are tuned properly

    // kF (Feedforward): Main power output based on target RPM
    // - This should get you ~90% of the way to target speed
    // - Tune this first by testing different RPMs and adjusting until close
    public static double kF = 0.000172;

    // kP (Proportional): Corrects remaining error
    // - Too low: slow to reach target, won't maintain speed under load
    // - Too high: oscillates/overshoots
    // - Tune after kF to eliminate steady-state error
    public static double kP = 0.0019;

    // kI (Integral): Eliminates long-term steady-state error
    // - Set to 0 for flywheels (usually not needed, can cause overshoot)
    // - Only increase if shooter consistently undershoots target by 50+ RPM
    public static double kI = 0.0;

    // kD (Derivative): Dampens oscillations
    // - Set to 0 for flywheels (makes control noisy, usually not needed)
    // - Only increase if you see oscillations around the target RPM
    public static double kD = 0.0;

    // PIDF output limits
    public static double MIN_POWER = 0.0;
    public static double MAX_POWER = 1.0;

    // Default target RPM for PIDF control
    public static double closeTargetRPM = 3700.0;
    public static double farTargetRPM = 5000.0;
    public static double tolRpm = 50.0;
    public static double tolRpm2 = 100.0;

    // ========== MOTOR CONFIGURATION ==========
    // TODO: VERIFY THIS VALUE! It's critical for accurate RPM calculation
    // Common values:
    //   - REV HD Hex Motor: 2240 ticks/rev
    //   - goBILDA 5202 (1:1): 537.7 ticks/rev
    //   - goBILDA 5203 (1:1): 537.7 ticks/rev
    //   - NeveRest 40: 1120 ticks/rev
    //   - NeveRest 60: 1680 ticks/rev
    // If using a gearbox/pulley, multiply by the ratio!
    public static double TICKS_PER_REV = 537.7;  // TODO: VERIFY THIS!

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