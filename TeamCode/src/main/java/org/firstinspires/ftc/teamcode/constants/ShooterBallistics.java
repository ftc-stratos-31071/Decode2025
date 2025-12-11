package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

/**
 * ShooterBallistics - Distance-based RPM and Hood calculations
 *
 * ==================== HOW TO TUNE SHOOTER BALLISTICS ====================
 *
 * OVERVIEW:
 * This class calculates the optimal RPM and hood angle based on distance
 * to the target. It uses mathematical models that you tune by adjusting
 * the coefficients below.
 *
 * STEP-BY-STEP TUNING PROCESS:
 *
 * 1. Run "TuneShooterBallistics" OpMode
 * 2. Open FTC Dashboard (http://192.168.43.1:8080/dash)
 * 3. Make sure PIDF is already tuned (see ShooterConstants.java)
 *
 * 4. TUNE RPM MODEL:
 *    Formula: RPM = RPM_A + RPM_B * distance + RPM_C * distance²
 *
 *    a) Start at a CLOSE distance (e.g., 1 meter)
 *    b) Adjust RPM_A until shots land accurately at close range
 *    c) Move to a FAR distance (e.g., 3 meters)
 *    d) Adjust RPM_B until shots land accurately at far range
 *    e) If shots are inconsistent at mid-range, adjust RPM_C
 *
 *    TIPS:
 *    - RPM_A is your "base" RPM (RPM at 0 distance)
 *    - RPM_B controls how much RPM increases per meter
 *    - RPM_C adds curvature (usually keep at 0 unless needed)
 *
 * 5. TUNE HOOD MODEL:
 *    Formula: Hood = HOOD_A + HOOD_B * distance
 *
 *    a) At CLOSE range, hood should be higher (steeper angle)
 *    b) At FAR range, hood should be lower (flatter angle)
 *    c) Adjust HOOD_A for close-range hood position
 *    d) Adjust HOOD_B for how much hood changes per meter (usually negative)
 *
 *    TIPS:
 *    - HOOD_A is hood position at 0 distance
 *    - HOOD_B is typically negative (hood gets flatter as distance increases)
 *
 * 6. TEST AT MULTIPLE DISTANCES:
 *    - Test at 1m, 2m, 3m, etc.
 *    - Fine-tune coefficients until accurate at all distances
 *
 * ==================================================================
 */
@Config
public class ShooterBallistics {

    // ----- RPM MODEL -----
    // RPM(d) = RPM_A + RPM_B*d + RPM_C*d²  (all tunable live via Dashboard)
    public static double RPM_A = 2600;   // Base RPM (RPM at 0 distance)
    public static double RPM_B = 700;    // Linear term (RPM increase per meter)
    public static double RPM_C = 0;      // Quadratic term (curvature, usually 0)

    public static double MIN_RPM = 2000; // Minimum allowed RPM
    public static double MAX_RPM = 6000; // Maximum allowed RPM

    // ----- HOOD MODEL -----
    // Hood(d) = HOOD_A + HOOD_B*d  (linear model)
    public static double HOOD_A = 0.60;  // Hood position at 0 distance (close = higher)
    public static double HOOD_B = -0.08; // Hood change per meter (negative = flattens with distance)

    public static double MIN_HOOD = 0.20; // Minimum hood position (flattest)
    public static double MAX_HOOD = 0.60; // Maximum hood position (steepest)

    // ----- DISTANCE LIMITS -----
    public static double MIN_DIST = 0.5; // Minimum distance we care about (meters)
    public static double MAX_DIST = 4.0; // Maximum distance we care about (meters)

    /**
     * Calculate target RPM based on distance to target
     * @param dMeters Distance to target in meters
     * @return Target RPM, clamped to MIN_RPM and MAX_RPM
     */
    public static double rpmFromDistance(double dMeters) {
        double d = clamp(dMeters, MIN_DIST, MAX_DIST);
        double rpm = RPM_A + RPM_B * d + RPM_C * d * d;
        return clamp(rpm, MIN_RPM, MAX_RPM);
    }

    /**
     * Calculate hood servo position based on distance to target
     * @param dMeters Distance to target in meters
     * @return Hood servo position, clamped to MIN_HOOD and MAX_HOOD
     */
    public static double hoodFromDistance(double dMeters) {
        double d = clamp(dMeters, MIN_DIST, MAX_DIST);
        double hood = HOOD_A + HOOD_B * d;
        return clamp(hood, MIN_HOOD, MAX_HOOD);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}

