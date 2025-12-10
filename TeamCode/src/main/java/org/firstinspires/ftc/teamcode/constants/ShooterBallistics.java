package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterBallistics {

    // ----- RPM MODEL -----
    // RPM(d) = A + B*d + C*d^2  (all tunable live)
    public static double RPM_A = 2600;   // base RPM
    public static double RPM_B = 700;    // linear term
    public static double RPM_C = 0;      // quadratic term (start at 0)

    public static double MIN_RPM = 2000;
    public static double MAX_RPM = 6000;

    // ----- HOOD MODEL -----
    // hood(d) ~ linear: closer = higher angle, further = flatter
    // hood(d) = H_A + H_B*d
    public static double HOOD_A = 0.60;  // hood at 0m (imaginary)
    public static double HOOD_B = -0.08; // how fast it flattens per meter

    public static double MIN_HOOD = 0.20;
    public static double MAX_HOOD = 0.60;

    // Clamp distance we care about
    public static double MIN_DIST = 0.5;
    public static double MAX_DIST = 4.0;

    public static double rpmFromDistance(double dMeters) {
        double d = clamp(dMeters, MIN_DIST, MAX_DIST);
        double rpm = RPM_A + RPM_B * d + RPM_C * d * d;
        return clamp(rpm, MIN_RPM, MAX_RPM);
    }

    public static double hoodFromDistance(double dMeters) {
        double d = clamp(dMeters, MIN_DIST, MAX_DIST);
        double hood = HOOD_A + HOOD_B * d;
        return clamp(hood, MIN_HOOD, MAX_HOOD);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}