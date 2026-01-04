package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TurretConstants {
    // FIXED: Increased PID values for proper encoder tick control
    // Previous values were tuned for degrees, but motor uses encoder ticks (1440 per rotation)
    public static double kP = 0.012;  // Increased from 0.00305 for better response
    public static double kI = 0.0;    // Keep at 0 to start
    public static double kD = 0.001; // Small D term to reduce oscillation
    public static double pos = 0.0;
}
