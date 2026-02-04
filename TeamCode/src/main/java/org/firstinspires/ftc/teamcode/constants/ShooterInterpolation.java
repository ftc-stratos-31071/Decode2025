package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

/**
 * ShooterInterpolation - Cubic Spline Interpolation for Distance-Based Shooting
 *
 * ==================== HOW INTERPOLATION WORKS ====================
 *
 * Instead of using simple physics equations (which don't account for real-world
 * factors like wheel compression, battery sag, ball slip, etc.), this class uses
 * EMPIRICAL DATA collected from actual field testing.
 *
 * You measure what RPM and hood angle work at various distances, and the system
 * uses cubic spline interpolation to smoothly fill in the gaps between your
 * calibration points.
 *
 * BENEFITS OF CUBIC SPLINE:
 * - Smooth transitions (no sharp changes like linear interpolation)
 * - Continuous first and second derivatives (smooth acceleration)
 * - Exact matches at calibration points
 * - More accurate than polynomial fitting
 *
 * ==================== HOW TO CALIBRATE ====================
 *
 * STEP 1: COLLECT DATA
 * ----------------------
 * For each distance you want to test:
 * 1. Position robot at measured distance from target
 * 2. Adjust RPM and hood angle until shots are accurate
 * 3. Record: [distance_inches, rpm, hood_position]
 * 4. Repeat for 5-8 different distances covering your shooting range
 *
 * STEP 2: ADD DATA TO TABLES
 * ---------------------------
 * Update the arrays below with your measured data:
 * - CALIBRATION_DISTANCES: Distance in inches (MUST be sorted, smallest to largest)
 * - CALIBRATION_RPM: Target RPM at each distance
 * - CALIBRATION_HOOD: Hood servo position at each distance
 *
 * STEP 3: TEST
 * -------------
 * 1. Run robot at various distances
 * 2. Check if interpolated values produce accurate shots
 * 3. Add more calibration points if needed (especially at problematic distances)
 *
 * TIPS:
 * - More calibration points = more accurate interpolation
 * - Focus points around your most common shooting distances
 * - If shots are inconsistent at a distance, add a calibration point there
 * - Test after battery changes or mechanical adjustments
 *
 * ==================================================================
 */
@Config
public class ShooterInterpolation {

    // ==================== CALIBRATION DATA ====================

    /**
     * CRITICAL: These arrays MUST be the same length!
     * Distances MUST be sorted in ascending order (smallest to largest)
     *
     * Example calibration data (REPLACE WITH YOUR ACTUAL MEASUREMENTS):
     * Distance (inches) | RPM  | Hood Position
     * ------------------|------|---------------
     *        40         | 2800 | 0.60 (steep)
     *        50         | 3000 | 0.55
     *        60         | 3200 | 0.50
     *        70         | 3400 | 0.45
     *        80         | 3650 | 0.40
     *        90         | 3900 | 0.35
     *       100         | 4200 | 0.30
     *       110         | 4500 | 0.25 (flat)
     */

    // TODO: REPLACE THESE WITH YOUR ACTUAL CALIBRATION DATA!
    public static double[] CALIBRATION_DISTANCES = {
        40.0,   // Close range
        50.0,
        60.0,
        70.0,   // Medium range
        80.0,
        90.0,
        100.0,  // Long range
        110.0
    };

    public static double[] CALIBRATION_RPM = {
        2800.0,  // Close shots need less speed
        3000.0,
        3200.0,
        3400.0,  // Medium range
        3650.0,
        3900.0,
        4200.0,  // Far shots need more speed
        4500.0
    };

    public static double[] CALIBRATION_HOOD = {
        0.60,    // Close = steep angle
        0.55,
        0.50,
        0.45,    // Medium range
        0.40,
        0.35,
        0.30,    // Far = flatter angle
        0.25
    };

    // ==================== SAFETY LIMITS ====================

    // Clamp outputs to prevent dangerous values
    public static double MIN_RPM = 2000.0;
    public static double MAX_RPM = 6000.0;
    public static double MIN_HOOD = 0.20;  // Flattest angle
    public static double MAX_HOOD = 0.65;  // Steepest angle

    // Distance limits (don't extrapolate beyond calibrated range)
    public static double MIN_DISTANCE = 35.0;   // Minimum shooting distance
    public static double MAX_DISTANCE = 120.0;  // Maximum shooting distance

    // ==================== INTERPOLATION OBJECTS ====================

    private static CubicSpline rpmSpline = null;
    private static CubicSpline hoodSpline = null;
    private static boolean initialized = false;

    /**
     * Initialize the cubic spline interpolators.
     * This builds the spline functions from calibration data.
     * Called automatically on first use.
     */
    private static void initializeSplines() {
        if (initialized) return;

        try {
            // Validate data
            if (!validateCalibrationData()) {
                throw new IllegalStateException("Invalid calibration data - check arrays!");
            }

            // Build cubic splines from calibration data
            rpmSpline = new CubicSpline(CALIBRATION_DISTANCES, CALIBRATION_RPM);
            hoodSpline = new CubicSpline(CALIBRATION_DISTANCES, CALIBRATION_HOOD);

            initialized = true;

        } catch (Exception e) {
            // If spline fails, fall back to simple linear model
            System.err.println("SPLINE INITIALIZATION FAILED: " + e.getMessage());
            e.printStackTrace();
            initialized = false;
        }
    }

    /**
     * Validate calibration data before building splines
     */
    private static boolean validateCalibrationData() {
        // Check array lengths match
        if (CALIBRATION_DISTANCES.length != CALIBRATION_RPM.length ||
            CALIBRATION_DISTANCES.length != CALIBRATION_HOOD.length) {
            System.err.println("ERROR: Calibration array lengths don't match!");
            return false;
        }

        // Need at least 2 points for interpolation
        if (CALIBRATION_DISTANCES.length < 2) {
            System.err.println("ERROR: Need at least 2 calibration points!");
            return false;
        }

        // Check distances are sorted (strictly increasing)
        for (int i = 1; i < CALIBRATION_DISTANCES.length; i++) {
            if (CALIBRATION_DISTANCES[i] <= CALIBRATION_DISTANCES[i-1]) {
                System.err.println("ERROR: Distances must be strictly increasing!");
                System.err.println("Problem at index " + i + ": " +
                    CALIBRATION_DISTANCES[i-1] + " -> " + CALIBRATION_DISTANCES[i]);
                return false;
            }
        }

        return true;
    }

    /**
     * Get target RPM for a given distance using cubic spline interpolation
     *
     * @param distanceInches Distance to target in inches
     * @return Target RPM, clamped to safe limits
     */
    public static double getRPMForDistance(double distanceInches) {
        // Lazy initialization
        if (!initialized) {
            initializeSplines();
        }

        // Clamp input distance to calibrated range (don't extrapolate)
        double clampedDistance = clamp(distanceInches, MIN_DISTANCE, MAX_DISTANCE);

        double rpm;
        if (initialized && rpmSpline != null) {
            // Use cubic spline interpolation
            try {
                rpm = rpmSpline.interpolate(clampedDistance);
            } catch (Exception e) {
                // Fallback to linear interpolation if spline fails
                rpm = linearInterpolate(CALIBRATION_DISTANCES, CALIBRATION_RPM, clampedDistance);
            }
        } else {
            // Fallback: linear interpolation
            rpm = linearInterpolate(CALIBRATION_DISTANCES, CALIBRATION_RPM, clampedDistance);
        }

        // Clamp output to safe range
        return clamp(rpm, MIN_RPM, MAX_RPM);
    }

    /**
     * Get hood servo position for a given distance using cubic spline interpolation
     *
     * @param distanceInches Distance to target in inches
     * @return Hood servo position (0.0-1.0), clamped to safe limits
     */
    public static double getHoodForDistance(double distanceInches) {
        // Lazy initialization
        if (!initialized) {
            initializeSplines();
        }

        // Clamp input distance to calibrated range
        double clampedDistance = clamp(distanceInches, MIN_DISTANCE, MAX_DISTANCE);

        double hood;
        if (initialized && hoodSpline != null) {
            // Use cubic spline interpolation
            try {
                hood = hoodSpline.interpolate(clampedDistance);
            } catch (Exception e) {
                // Fallback to linear interpolation if spline fails
                hood = linearInterpolate(CALIBRATION_DISTANCES, CALIBRATION_HOOD, clampedDistance);
            }
        } else {
            // Fallback: linear interpolation
            hood = linearInterpolate(CALIBRATION_DISTANCES, CALIBRATION_HOOD, clampedDistance);
        }

        // Clamp output to safe range
        return clamp(hood, MIN_HOOD, MAX_HOOD);
    }

    /**
     * Force re-initialization of splines (call after updating calibration data)
     */
    public static void forceReinitialize() {
        initialized = false;
        rpmSpline = null;
        hoodSpline = null;
        initializeSplines();
    }

    /**
     * Check if splines are properly initialized
     */
    public static boolean isInitialized() {
        return initialized;
    }

    /**
     * Get diagnostic info about calibration status
     */
    public static String getCalibrationStatus() {
        if (!initialized) {
            return "Not initialized";
        }

        int numPoints = CALIBRATION_DISTANCES.length;
        double minDist = CALIBRATION_DISTANCES[0];
        double maxDist = CALIBRATION_DISTANCES[numPoints - 1];

        return String.format("Initialized: %d points, range %.1f-%.1f inches",
                           numPoints, minDist, maxDist);
    }

    // ==================== UTILITY METHODS ====================

    /**
     * Simple linear interpolation fallback
     */
    private static double linearInterpolate(double[] xValues, double[] yValues, double x) {
        // Find bracketing indices
        int i = 0;
        while (i < xValues.length - 1 && xValues[i + 1] < x) {
            i++;
        }

        // Handle edge cases
        if (i >= xValues.length - 1) {
            return yValues[yValues.length - 1];
        }

        // Linear interpolation formula
        double x0 = xValues[i];
        double x1 = xValues[i + 1];
        double y0 = yValues[i];
        double y1 = yValues[i + 1];

        double t = (x - x0) / (x1 - x0);
        return y0 + t * (y1 - y0);
    }

    /**
     * Clamp value to range
     */
    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    // ==================== CUBIC SPLINE IMPLEMENTATION ====================

    /**
     * Custom cubic spline implementation (natural spline with zero second derivatives at endpoints)
     */
    private static class CubicSpline {
        private final double[] x;  // Known x values (distances)
        private final double[] y;  // Known y values (RPM or hood)
        private final double[] a, b, c, d;  // Cubic coefficients for each segment
        private final int n;  // Number of data points

        public CubicSpline(double[] x, double[] y) {
            this.n = x.length;
            this.x = x.clone();
            this.y = y.clone();

            // Allocate coefficient arrays
            this.a = new double[n];
            this.b = new double[n];
            this.c = new double[n];
            this.d = new double[n];

            // Compute spline coefficients
            computeCoefficients();
        }

        /**
         * Compute cubic spline coefficients using natural spline conditions
         */
        private void computeCoefficients() {
            // For n points, we have n-1 cubic polynomials
            int segments = n - 1;

            // Step 1: Compute h[] (distances between consecutive x values)
            double[] h = new double[segments];
            for (int i = 0; i < segments; i++) {
                h[i] = x[i + 1] - x[i];
            }

            // Step 2: Set up tridiagonal system for second derivatives
            // Natural spline: c[0] = 0, c[n-1] = 0
            double[] alpha = new double[segments];
            for (int i = 1; i < segments; i++) {
                alpha[i] = (3.0 / h[i]) * (y[i + 1] - y[i]) -
                          (3.0 / h[i - 1]) * (y[i] - y[i - 1]);
            }

            // Step 3: Solve tridiagonal system using Thomas algorithm
            double[] l = new double[n];
            double[] mu = new double[n];
            double[] z = new double[n];

            l[0] = 1.0;
            mu[0] = 0.0;
            z[0] = 0.0;

            for (int i = 1; i < segments; i++) {
                l[i] = 2.0 * (x[i + 1] - x[i - 1]) - h[i - 1] * mu[i - 1];
                mu[i] = h[i] / l[i];
                z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
            }

            l[segments] = 1.0;
            z[segments] = 0.0;
            c[segments] = 0.0;

            // Step 4: Back-substitution
            for (int j = segments - 1; j >= 0; j--) {
                c[j] = z[j] - mu[j] * c[j + 1];
                b[j] = (y[j + 1] - y[j]) / h[j] - h[j] * (c[j + 1] + 2.0 * c[j]) / 3.0;
                d[j] = (c[j + 1] - c[j]) / (3.0 * h[j]);
                a[j] = y[j];
            }
        }

        /**
         * Interpolate value at given x using cubic spline
         */
        public double interpolate(double xVal) {
            // Find the segment that contains xVal
            int i = 0;
            for (i = 0; i < n - 1; i++) {
                if (xVal <= x[i + 1]) {
                    break;
                }
            }

            // Clamp to valid segment
            if (i >= n - 1) {
                i = n - 2;
            }

            // Evaluate cubic polynomial: a + b*dx + c*dx^2 + d*dx^3
            double dx = xVal - x[i];
            return a[i] + b[i] * dx + c[i] * dx * dx + d[i] * dx * dx * dx;
        }
    }
}
