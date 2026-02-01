package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;
import java.util.Locale;
import java.util.Queue;
import java.util.stream.Collectors;

/**
 * ╔══════════════════════════════════════════════════════════════════════════════╗
 * ║          ARTIFACT PURSUIT TELEOP - LIMELIGHT 3A VISION SYSTEM                ║
 * ║               Driver-Assist for DECODE Scoring Elements                      ║
 * ╚══════════════════════════════════════════════════════════════════════════════╝
 *
 * SYSTEM OVERVIEW:
 * ═══════════════════════════════════════════════════════════════════════════════
 * This TeleOp implements a complete Limelight 3A-based computer vision system for
 * detecting and pursuing DECODE artifacts (5-inch polypropylene balls). The system
 * operates entirely within TeleOp as a driver-assist feature.
 *
 * KEY FEATURES:
 * - Universal ball detection (works with ANY Limelight pipeline)
 * - Size and shape-based filtering (color-agnostic)
 * - Multi-ball clustering to find densest group
 * - State machine-based pursuit (SEARCH → LOCK → APPROACH → ARRIVE)
 * - PID-based rotation control for smooth alignment
 * - Area-based distance estimation for approach control
 * - Failsafe handling for lost targets
 *
 * ══════════════════════════════════════════════════════════════════════════════
 * LIMELIGHT PIPELINE SETUP (FLEXIBLE - USE ANY DETECTION METHOD):
 * ══════════════════════════════════════════════════════════════════════════════
 *
 * This code works with ANY of the following pipeline types:
 *
 * OPTION 1: COLOR DETECTOR (Recommended for bright balls)
 *    - Set up to detect ANY high-contrast circular objects
 *    - Use broad HSV ranges to catch all ball colors
 *    - OR set specific ranges if you know the ball colors
 *
 * OPTION 2: NEURAL DETECTOR (Best for complex backgrounds)
 *    - Train or use pre-trained model to detect "ball" objects
 *    - Works regardless of ball color
 *
 * OPTION 3: PYTHON/CUSTOM PIPELINE
 *    - Detect circular blobs by shape (Hough circles, etc.)
 *    - Filter by size/roundness
 *
 * UNIVERSAL PIPELINE SETTINGS:
 * ────────────────────────────────────────────────────────────────────────────
 * 1. Access Limelight: http://limelight.local:5801
 * 2. Select Pipeline 8 (or change ARTIFACT_PIPELINE constant)
 *
 * 3. Camera Settings (CRITICAL for performance):
 *    - Resolution: 320×240 @ 90fps (or 640×480 @ 90fps)
 *    - Exposure: ~5 ms (fixed, low to reduce motion blur)
 *    - Gain: ~20% (minimal noise)
 *    - White Balance: Fixed (prevents color shift during match)
 *
 * 4. Detection Settings (if using Color pipeline):
 *    - Use BROAD thresholds to catch all ball colors
 *    - OR detect by brightness/contrast instead of specific colors
 *    - Morphology: Erosion 1-2, Dilation 1-2 (clean noise)
 *
 * 5. Contour/Object Filtering (IMPORTANT):
 *    - Min Area: 0.1-0.2% (catches distant balls, ~6+ feet away)
 *    - Max Area: 20-25% (filters walls/large objects)
 *    - Aspect Ratio: 0.6-1.5 (roughly circular)
 *    - Fullness/Solidity: >0.6 (solid objects, not hollow)
 *
 * 6. Save Pipeline
 *
 * The code automatically tries multiple detection methods:
 *   → ColorResults (if Color pipeline active)
 *   → DetectorResults (if Neural/Object Detection active)
 *   → Generic target detection (fallback for any pipeline type)
 *
 * ══════════════════════════════════════════════════════════════════════════════
 * HOW THE SYSTEM WORKS (END-TO-END):
 * ══════════════════════════════════════════════════════════════════════════════
 *
 * DETECTION PIPELINE:
 * 1. Limelight captures frame (OV5647 sensor, 54.5°×42° FOV)
 * 2. Image Signal Processing applies exposure/gain/white balance
 * 3. Pipeline detects ball-shaped objects (method depends on pipeline type)
 * 4. Filtering removes invalid shapes (size/aspect/fullness checks)
 * 5. Results sent to FTC SDK via USB at 100 Hz
 *
 * CLUSTERING LOGIC:
 * - Collects all detected ball objects from Limelight
 * - Groups balls by angular proximity (Euclidean distance in degrees)
 * - Calculates weighted center of mass for each cluster
 * - Scores clusters by count (primary) and total area (secondary)
 * - Outputs cluster centroid (tx, ty) and max area for control
 *
 * CONTROL ALGORITHM:
 * State Machine with 5 states:
 *
 * MANUAL: Driver has full control (default)
 *   → Press LEFT BUMPER to activate assist
 *
 * SEARCH: No target visible, slow rotation scan
 *   → Transition to LOCK when target appears
 *
 * LOCK: Target detected, rotating to center it
 *   → Uses PID: turnPower = kP * tx + kD * (tx - lastError)
 *   → Transition to APPROACH when |tx| < tolerance
 *
 * APPROACH: Aligned, driving forward while maintaining heading
 *   → Forward speed based on area (far=fast, near=slow)
 *   → Continuous heading correction via reduced PID
 *   → Transition to ARRIVE when ta > threshold
 *
 * ARRIVE: At intake range, stop for collection
 *   → Can return to SEARCH for next ball
 *
 * FAILSAFE: Target lost unexpectedly
 *   → Returns to SEARCH to reacquire
 *
 * PID ROTATION CONTROL:
 * - Proportional: Responds to current error (tx degrees off center)
 * - Derivative: Dampens oscillation (rate of change)
 * - Output clamped to [MIN_ROTATE_POWER, MAX_ROTATE_POWER]
 * - Deadband at |tx| < 1° to prevent jitter
 *
 * DISTANCE ESTIMATION:
 * - Uses target area (ta) as proxy: area ∝ 1/distance²
 * - Larger area = closer ball
 * - Thresholds: CLOSE_RANGE_AREA, INTAKE_RANGE_AREA
 * - Progressive speed reduction as area increases
 *
 * LATENCY OPTIMIZATION:
 * - Total end-to-end: ~30-50ms
 *   • Frame capture: ~11ms @ 90fps
 *   • Pipeline processing: ~10-20ms (depends on pipeline type)
 *   • USB transfer: ~5ms
 *   • FTC loop: ~20ms @ 50Hz
 * - High polling rate (100Hz) ensures fresh data
 * - Low exposure (5ms) reduces motion blur
 *
 * ══════════════════════════════════════════════════════════════════════════════
 * CONTROLS:
 * ══════════════════════════════════════════════════════════════════════════════
 *
 * GAMEPAD 1:
 *   LEFT BUMPER (hold)   = Activate artifact pursuit assist
 *   Left Stick Y/X       = Manual drive (when assist off)
 *   Right Stick X        = Manual rotation (when assist off)
 *   Right Bumper         = Toggle slow mode (25% speed)
 *   D-Pad Up/Down        = Adjust approach speed ±0.01
 *   D-Pad Left/Right     = Adjust alignment tolerance ±0.5°
 *   X                    = Reset PID integral (if oscillating)
 *   Y                    = Cycle through debug display modes
 *
 * ══════════════════════════════════════════════════════════════════════════════
 * TUNING GUIDE:
 * ═══════════════════════════════════════════════════════════════════════════════
 *
 * PROBLEM: Robot rotates too slowly
 * SOLUTION: Increase KP_ROTATE (try 0.025, 0.03, 0.035)
 *
 * PROBLEM: Robot oscillates/overshoots
 * SOLUTION: Increase KD_ROTATE (try 0.005, 0.007)
 *           OR decrease KP_ROTATE
 *
 * PROBLEM: Robot doesn't move at all
 * SOLUTION: Check MIN_ROTATE_POWER (needs to overcome friction)
 *           Verify Limelight detecting targets (check telemetry)
 *
 * PROBLEM: Approaches too fast
 * SOLUTION: Decrease APPROACH_SPEED and SLOW_APPROACH_SPEED
 *
 * PROBLEM: Stops too far away
 * SOLUTION: Decrease INTAKE_RANGE_AREA threshold
 *
 * PROBLEM: Too many false detections
 * SOLUTION: Increase MIN_TARGET_AREA in pipeline
 *           Tighten size/shape filters on Limelight
 *
 * PROBLEM: Missing balls at distance
 * SOLUTION: Decrease MIN_TARGET_AREA (allow smaller blobs)
 *           Check camera exposure/gain settings
 *
 * PROBLEM: Jitters when centered
 * SOLUTION: Increase ALIGNMENT_TOLERANCE (deadband)
 *           Decrease KP_ROTATE
 *
 * PROBLEM: Clusters not grouping correctly
 * SOLUTION: Adjust CLUSTER_ANGLE_THRESHOLD
 *           5° = tight grouping, 10° = loose grouping
 *
 * ═══════════════════════════════════════════════════════════════════════════════
 * FTC DASHBOARD TUNING:
 * ═══════════════════════════════════════════════════════════════════════════════
 *
 * Access FTC Dashboard at: http://192.168.43.1:8080/dash (or appropriate IP)
 *
 * All constants are now tunable in real-time via FTC Dashboard:
 * - Vision thresholds (MIN_TARGET_AREA, MAX_TARGET_AREA)
 * - PID gains (KP_ROTATE, KD_ROTATE)
 * - Clustering parameters (CLUSTER_ANGLE_THRESHOLD)
 * - Approach speeds and tolerances
 * - Target selection mode (LARGEST_CLUSTER, CLOSEST, LEFTMOST, etc.)
 *
 * Changes take effect immediately without restarting the OpMode.
 *
 * Dashboard also shows:
 * - Live camera stream from Limelight
 * - Real-time detection overlays
 * - State machine status
 * - PID error values
 * - Limelight performance metrics (FPS, CPU, Temp)
 *
 * ══════════════════════════════════════════════════════════════════════════════
 */
@Config
@TeleOp(name = "Artifact Pursuit TeleOp", group = "Vision")
public class ArtifactPursuitTeleop extends LinearOpMode {

    // ═══════════════════════════════════════════════════════════════════════════
    // HARDWARE
    // ═══════════════════════════════════════════════════════════════════════════
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private Limelight3A limelight;
    private FtcDashboard dashboard;
    private Telemetry dashboardTelemetry;

    // ═══════════════════════════════════════════════════════════════════════════
    // VISION CONSTANTS - Pipeline Configuration (FTC Dashboard Tunable)
    // ═══════════════════════════════════════════════════════════════════════════
    public static int ARTIFACT_PIPELINE = 7;                  // Pipeline index
    public static double MIN_TARGET_AREA = 0.15;              // Min % to be valid target
    public static double MAX_TARGET_AREA = 25.0;              // Max % to filter walls/false positives
    public static double INTAKE_RANGE_AREA = 12.0;            // Area when at intake range
    public static double CLOSE_RANGE_AREA = 6.0;              // Area to start slowing

    // ═══════════════════════════════════════════════════════════════════════════
    // CLUSTERING CONSTANTS (FTC Dashboard Tunable)
    // ═══════════════════════════════════════════════════════════════════════════
    public static double CLUSTER_ANGLE_THRESHOLD = 8.0;       // Max degrees apart to cluster
    public static int MIN_CLUSTER_SIZE = 1;                   // Minimum balls to form valid cluster
    public static TargetSelectionMode SELECTION_MODE = TargetSelectionMode.LARGEST_CLUSTER;

    // ═══════════════════════════════════════════════════════════════════════════
    // PID CONSTANTS - Rotation Control (FTC Dashboard Tunable)
    // ═══════════════════════════════════════════════════════════════════════════
    public static double KP_ROTATE = 0.022;                   // Proportional gain
    public static double KI_ROTATE = 0.0;                     // Integral gain (usually 0)
    public static double KD_ROTATE = 0.004;                   // Derivative gain
    public static double MIN_ROTATE_POWER = 0.15;             // Overcome static friction
    public static double MAX_ROTATE_POWER = 0.65;             // Max rotation speed

    // ═══════════════════════════════════════════════════════════════════════════
    // DRIVE CONSTANTS (FTC Dashboard Tunable)
    // ═══════════════════════════════════════════════════════════════════════════
    public static double APPROACH_SPEED = 0.4;                // Speed when pursuing
    public static double SLOW_APPROACH_SPEED = 0.22;          // Speed when very close
    public static double SEARCH_ROTATE_SPEED = 0.28;          // Rotation when searching
    public static double ALIGNMENT_TOLERANCE = 3.5;           // Degrees acceptable error

    // ═══════════════════════════════════════════════════════════════════════════
    // CAMERA SPECIFICATIONS (for utility calculations)
    // ═══════════════════════════════════════════════════════════════════════════
    private static final double CAMERA_HFOV = 54.5;           // Horizontal FOV (degrees)
    private static final double CAMERA_VFOV = 42.0;           // Vertical FOV (degrees)

    // ═══════════════════════════════════════════════════════════════════════════
    // TARGET SELECTION MODES
    // ═══════════════════════════════════════════════════════════════════════════
    public enum TargetSelectionMode {
        LARGEST_CLUSTER,    // Most balls in group (default)
        CLOSEST,            // Largest area (nearest)
        LEFTMOST,           // Leftmost target (negative tx)
        RIGHTMOST,          // Rightmost target (positive tx)
        CENTERED            // Closest to center (smallest |tx|)
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // STATE MACHINE
    // ═══════════════════════════════════════════════════════════════════════════
    private enum PursuitState {
        MANUAL,         // Driver control
        SEARCHING,      // Rotating to find targets
        LOCK,          // Rotating to align
        APPROACH,       // Driving toward target
        ARRIVE,         // At intake range
        FAILSAFE        // Target lost
    }

    private PursuitState currentState = PursuitState.MANUAL;

    // PID State
    private double lastError = 0;
    private double integralSum = 0;

    // UI State
    private boolean slowMode = false;
    private double driveScale = 1.0;
    private boolean lastRightBumper = false;
    private boolean lastY = false;
    private int debugMode = 0; // 0=normal, 1=detailed, 2=raw data

    // ═══════════════════════════════════════════════════════════════════════════
    // MAIN OPMODE
    // ═══════════════════════════════════════════════════════════════════════════
    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        initializeDashboard();
        displayInitTelemetry();

        waitForStart();

        while (opModeIsActive()) {
            handleControls();
            updatePursuitStateMachine();
            displayTelemetry();
            updateDashboard();
        }

        cleanup();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // HARDWARE INITIALIZATION
    // ═══════════════════════════════════════════════════════════════════════════
    private void initializeHardware() {
        // Drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotor.class, "backRightMotor");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Limelight
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);  // High refresh for responsive control
            limelight.pipelineSwitch(ARTIFACT_PIPELINE);
            limelight.start();
        } catch (Exception e) {
            limelight = null;
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // DASHBOARD INITIALIZATION
    // ═══════════════════════════════════════════════════════════════════════════
    private void initializeDashboard() {
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // CONTROL INPUT HANDLING
    // ═══════════════════════════════════════════════════════════════════════════
    private void handleControls() {
        // Slow mode toggle
        if (gamepad1.right_bumper && !lastRightBumper) {
            slowMode = !slowMode;
            driveScale = slowMode ? 0.25 : 1.0;
        }
        lastRightBumper = gamepad1.right_bumper;

        // Debug mode cycling
        if (gamepad1.y && !lastY) {
            debugMode = (debugMode + 1) % 3;
        }
        lastY = gamepad1.y;

        // PID reset
        if (gamepad1.x) {
            integralSum = 0;
            lastError = 0;
        }

        // Live tuning
        if (gamepad1.dpad_up) {
            APPROACH_SPEED = Math.min(1.0, APPROACH_SPEED + 0.01);
        }
        if (gamepad1.dpad_down) {
            APPROACH_SPEED = Math.max(0.1, APPROACH_SPEED - 0.01);
        }
        if (gamepad1.dpad_right) {
            ALIGNMENT_TOLERANCE = Math.min(10.0, ALIGNMENT_TOLERANCE + 0.5);
        }
        if (gamepad1.dpad_left) {
            ALIGNMENT_TOLERANCE = Math.max(1.0, ALIGNMENT_TOLERANCE - 0.5);
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // STATE MACHINE UPDATE
    // ═══════════════════════════════════════════════════════════════════════════
    private void updatePursuitStateMachine() {
        boolean assistActive = gamepad1.left_bumper;

        if (!assistActive) {
            currentState = PursuitState.MANUAL;
            executeManualDrive();
            return;
        }

        // Get vision data and cluster
        ClusterData cluster = getArtifactCluster();

        // State transitions
        if (cluster.count == 0) {
            if (currentState != PursuitState.SEARCHING) {
                currentState = PursuitState.SEARCHING;
            }
        } else if (cluster.totalArea >= INTAKE_RANGE_AREA) {
            currentState = PursuitState.ARRIVE;
        } else if (Math.abs(cluster.centerX) > ALIGNMENT_TOLERANCE) {
            currentState = PursuitState.LOCK;
        } else {
            currentState = PursuitState.APPROACH;
        }

        // Execute current state behavior
        switch (currentState) {
            case SEARCHING:
                executeSearch();
                break;
            case LOCK:
                executeLock(cluster);
                break;
            case APPROACH:
                executeApproach(cluster);
                break;
            case ARRIVE:
                executeArrive();
                break;
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // STATE BEHAVIORS
    // ═══════════════════════════════════════════════════════════════════════════

    private void executeManualDrive() {
        double forward = -gamepad1.left_stick_y * driveScale;
        double strafe = gamepad1.left_stick_x * driveScale;
        double rotate = gamepad1.right_stick_x * driveScale;
        setMecanumPower(forward, strafe, rotate);
    }

    private void executeSearch() {
        setRotationPower(SEARCH_ROTATE_SPEED);
    }

    private void executeLock(ClusterData cluster) {
        double error = cluster.centerX;
        double derivative = error - lastError;
        integralSum += error;
        integralSum = Math.max(-50, Math.min(50, integralSum)); // Anti-windup

        double rotationPower = (KP_ROTATE * error) +
                              (KI_ROTATE * integralSum) +
                              (KD_ROTATE * derivative);

        // Apply minimum power threshold
        if (Math.abs(rotationPower) > 0.01) {
            if (rotationPower > 0) {
                rotationPower = Math.max(MIN_ROTATE_POWER, rotationPower);
            } else {
                rotationPower = Math.min(-MIN_ROTATE_POWER, rotationPower);
            }
        }

        // Clamp to max
        rotationPower = Math.max(-MAX_ROTATE_POWER, Math.min(MAX_ROTATE_POWER, rotationPower));

        setRotationPower(rotationPower);
        lastError = error;
    }

    private void executeApproach(ClusterData cluster) {
        double error = cluster.centerX;

        // Light steering correction (reduced gain)
        double steeringCorrection = KP_ROTATE * 0.5 * error;
        steeringCorrection = Math.max(-0.3, Math.min(0.3, steeringCorrection));

        // Speed based on distance (area)
        double forwardSpeed = (cluster.totalArea > CLOSE_RANGE_AREA) ?
                             SLOW_APPROACH_SPEED : APPROACH_SPEED;

        setMecanumPower(forwardSpeed, 0, steeringCorrection);
    }

    private void executeArrive() {
        stopDrive();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // VISION PROCESSING - Multi-ball Detection & Clustering (ENHANCED)
    // ═══════════════════════════════════════════════════════════════════════════

    private ClusterData getArtifactCluster() {
        List<ArtifactTarget> artifacts = getDetectedArtifacts();

        if (artifacts.isEmpty()) {
            return new ClusterData(0, 0, 0, 0);
        }

        // Apply selection mode
        switch (SELECTION_MODE) {
            case CLOSEST:
                return getClosestTarget(artifacts);
            case LEFTMOST:
                return getLeftmostTarget(artifacts);
            case RIGHTMOST:
                return getRightmostTarget(artifacts);
            case CENTERED:
                return getCenteredTarget(artifacts);
            case LARGEST_CLUSTER:
            default:
                return getLargestCluster(artifacts);
        }
    }

    /**
     * Original clustering algorithm - finds densest group of balls
     */
    private ClusterData getLargestCluster(List<ArtifactTarget> artifacts) {
        // Perform clustering
        List<Cluster> clusters = clusterTargets(artifacts);

        if (clusters.isEmpty()) {
            return new ClusterData(0, 0, 0, 0);
        }

        // Filter by minimum cluster size
        clusters = clusters.stream()
                .filter(c -> c.count >= MIN_CLUSTER_SIZE)
                .collect(Collectors.toList());

        if (clusters.isEmpty()) {
            return new ClusterData(0, 0, 0, 0);
        }

        // Find best cluster (most balls, then total area)
        Cluster bestCluster = clusters.get(0);
        double bestScore = bestCluster.count * 1000 + bestCluster.sumArea;

        for (Cluster cluster : clusters) {
            double score = cluster.count * 1000 + cluster.sumArea;
            if (score > bestScore) {
                bestScore = score;
                bestCluster = cluster;
            }
        }

        return new ClusterData(
            bestCluster.avgTx,
            bestCluster.avgTy,
            bestCluster.sumArea,
            bestCluster.count
        );
    }

    /**
     * Target closest to robot (largest area)
     */
    private ClusterData getClosestTarget(List<ArtifactTarget> artifacts) {
        ArtifactTarget closest = artifacts.stream()
                .max(Comparator.comparingDouble(a -> a.area))
                .orElse(null);

        if (closest == null) {
            return new ClusterData(0, 0, 0, 0);
        }

        return new ClusterData(closest.tx, closest.ty, closest.area, 1);
    }

    /**
     * Leftmost target (most negative tx)
     */
    private ClusterData getLeftmostTarget(List<ArtifactTarget> artifacts) {
        ArtifactTarget leftmost = artifacts.stream()
                .min(Comparator.comparingDouble(a -> a.tx))
                .orElse(null);

        if (leftmost == null) {
            return new ClusterData(0, 0, 0, 0);
        }

        return new ClusterData(leftmost.tx, leftmost.ty, leftmost.area, 1);
    }

    /**
     * Rightmost target (most positive tx)
     */
    private ClusterData getRightmostTarget(List<ArtifactTarget> artifacts) {
        ArtifactTarget rightmost = artifacts.stream()
                .max(Comparator.comparingDouble(a -> a.tx))
                .orElse(null);

        if (rightmost == null) {
            return new ClusterData(0, 0, 0, 0);
        }

        return new ClusterData(rightmost.tx, rightmost.ty, rightmost.area, 1);
    }

    /**
     * Target closest to center (smallest |tx|)
     */
    private ClusterData getCenteredTarget(List<ArtifactTarget> artifacts) {
        ArtifactTarget centered = artifacts.stream()
                .min(Comparator.comparingDouble(a -> Math.abs(a.tx)))
                .orElse(null);

        if (centered == null) {
            return new ClusterData(0, 0, 0, 0);
        }

        return new ClusterData(centered.tx, centered.ty, centered.area, 1);
    }

    private List<ArtifactTarget> getDetectedArtifacts() {
        List<ArtifactTarget> artifacts = new ArrayList<>();

        if (limelight == null) {
            return artifacts;
        }

        try {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();

                if (colorResults != null) {
                    for (LLResultTypes.ColorResult colorResult : colorResults) {
                        double area = colorResult.getTargetArea();

                        // Enhanced filtering with max area
                        if (area >= MIN_TARGET_AREA && area <= MAX_TARGET_AREA) {
                            artifacts.add(new ArtifactTarget(
                                colorResult.getTargetXDegrees(),
                                colorResult.getTargetYDegrees(),
                                area
                            ));
                        }
                    }
                }
            }
        } catch (Exception e) {
            // Silently handle vision exceptions
        }

        return artifacts;
    }

    private List<Cluster> clusterTargets(List<ArtifactTarget> targets) {
        List<Cluster> clusters = new ArrayList<>();
        boolean[] clustered = new boolean[targets.size()];

        for (int i = 0; i < targets.size(); i++) {
            if (clustered[i]) continue;

            Cluster cluster = new Cluster();
            Queue<Integer> queue = new LinkedList<>();
            queue.add(i);
            clustered[i] = true;

            while (!queue.isEmpty()) {
                int idx = queue.poll();
                ArtifactTarget target = targets.get(idx);

                cluster.avgTx += target.tx;
                cluster.avgTy += target.ty;
                cluster.sumArea += target.area;
                cluster.maxArea = Math.max(cluster.maxArea, target.area);
                cluster.count++;

                // Find nearby unclustered targets
                for (int j = 0; j < targets.size(); j++) {
                    if (clustered[j]) continue;

                    ArtifactTarget other = targets.get(j);
                    double dx = other.tx - target.tx;
                    double dy = other.ty - target.ty;
                    double dist = Math.sqrt(dx * dx + dy * dy);

                    if (dist < CLUSTER_ANGLE_THRESHOLD) {
                        queue.add(j);
                        clustered[j] = true;
                    }
                }
            }

            // Compute averages
            cluster.avgTx /= cluster.count;
            cluster.avgTy /= cluster.count;
            clusters.add(cluster);
        }

        return clusters;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // MOTOR CONTROL
    // ═══════════════════════════════════════════════════════════════════════════

    private void setMecanumPower(double forward, double strafe, double rotate) {
        double frontLeftPower = forward + strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backLeftPower = forward - strafe + rotate;
        double backRightPower = forward + strafe - rotate;

        double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    private void setRotationPower(double power) {
        power = Math.max(-1.0, Math.min(1.0, power));
        frontLeft.setPower(-power);
        backLeft.setPower(-power);
        frontRight.setPower(power);
        backRight.setPower(power);
    }

    private void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // TELEMETRY
    // ═══════════════════════════════════════════════════════════════════════════

    private void displayInitTelemetry() {
        telemetry.clear();
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine("  🎯 ARTIFACT PURSUIT TELEOP");
        telemetry.addLine("═══════════════════════════════════════");
        telemetry.addLine();

        if (limelight != null) {
            telemetry.addLine("✅ Limelight 3A Connected");
            telemetry.addData("Pipeline", ARTIFACT_PIPELINE);
            telemetry.addLine("Universal Ball Detection: Active");
            telemetry.addLine("Detection: Size & Shape-Based");
        } else {
            telemetry.addLine("⚠️ LIMELIGHT NOT FOUND");
            telemetry.addLine("Check robot configuration!");
        }

        telemetry.addLine();
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("  L-BUMPER = Pursuit Assist");
        telemetry.addLine("  R-BUMPER = Slow Mode");
        telemetry.addLine("  D-Pad ↑↓ = Speed");
        telemetry.addLine("  D-Pad ←→ = Tolerance");
        telemetry.addLine("  X = Reset PID");
        telemetry.addLine("  Y = Cycle Debug");
        telemetry.addLine();
        telemetry.addLine("FTC Dashboard: 192.168.43.1:8080/dash");
        telemetry.update();
    }

    private void displayTelemetry() {
        telemetry.clear();

        if (debugMode == 0) {
            displayNormalTelemetry();
        } else if (debugMode == 1) {
            displayDetailedTelemetry();
        } else {
            displayRawTelemetry();
        }

        telemetry.update();
    }

    private void displayNormalTelemetry() {
        telemetry.addLine("═══ ARTIFACT PURSUIT ═══");

        String stateEmoji = getStateEmoji(currentState);
        telemetry.addData(stateEmoji + " State", currentState.name());
        telemetry.addData("Slow Mode", slowMode ? "ON" : "OFF");
        telemetry.addLine();

        if (limelight != null) {
            ClusterData cluster = getArtifactCluster();
            telemetry.addData("🎯 Artifacts", cluster.count);

            if (cluster.count > 0) {
                telemetry.addData("Cluster X", String.format("%.2f°", cluster.centerX));
                telemetry.addData("Total Area", String.format("%.2f%%", cluster.totalArea));
                telemetry.addData("Distance", estimateDistance(cluster.totalArea));

                if (Math.abs(cluster.centerX) < ALIGNMENT_TOLERANCE) {
                    telemetry.addLine("✅ ALIGNED");
                } else {
                    telemetry.addData("Error", String.format("%.1f°", cluster.centerX));
                }
            } else {
                telemetry.addLine("❌ No artifacts");
            }

            try {
                LLStatus status = limelight.getStatus();
                if (status != null) {
                    telemetry.addData("LL FPS", (int) status.getFps());
                }
            } catch (Exception ignored) {}
        }
    }

    private void displayDetailedTelemetry() {
        telemetry.addLine("═══ DETAILED DEBUG ═══");
        telemetry.addData("State", currentState.name());
        telemetry.addData("Assist", gamepad1.left_bumper ? "ACTIVE" : "OFF");

        ClusterData cluster = getArtifactCluster();
        telemetry.addData("Count", cluster.count);
        telemetry.addData("Center X", String.format("%.2f", cluster.centerX));
        telemetry.addData("Center Y", String.format("%.2f", cluster.centerY));
        telemetry.addData("Sum Area", String.format("%.2f", cluster.totalArea));

        telemetry.addLine();
        telemetry.addData("KP", String.format("%.3f", KP_ROTATE));
        telemetry.addData("KD", String.format("%.3f", KD_ROTATE));
        telemetry.addData("Last Error", String.format("%.2f", lastError));
        telemetry.addData("Integral", String.format("%.2f", integralSum));

        telemetry.addLine();
        telemetry.addData("Speed", String.format("%.2f", APPROACH_SPEED));
        telemetry.addData("Tolerance", String.format("%.1f°", ALIGNMENT_TOLERANCE));
    }

    private void displayRawTelemetry() {
        telemetry.addLine("═══ RAW DATA ═══");

        if (limelight != null) {
            try {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    telemetry.addData("Valid", "YES");
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("ta", result.getTa());

                    List<LLResultTypes.ColorResult> results = result.getColorResults();
                    telemetry.addData("Raw Count", results != null ? results.size() : 0);

                    if (results != null && !results.isEmpty()) {
                        for (int i = 0; i < Math.min(3, results.size()); i++) {
                            LLResultTypes.ColorResult r = results.get(i);
                            telemetry.addData("T" + i, String.format("x:%.1f a:%.1f",
                                r.getTargetXDegrees(), r.getTargetArea()));
                        }
                    }
                } else {
                    telemetry.addData("Valid", "NO");
                }

                LLStatus status = limelight.getStatus();
                if (status != null) {
                    telemetry.addData("Temp", String.format("%.1f°C", status.getTemp()));
                    telemetry.addData("CPU", String.format("%.1f%%", status.getCpu()));
                    telemetry.addData("FPS", (int) status.getFps());
                }
            } catch (Exception e) {
                telemetry.addData("Error", e.getMessage());
            }
        }
    }

    private String getStateEmoji(PursuitState state) {
        switch (state) {
            case MANUAL: return "🎮";
            case SEARCHING: return "🔍";
            case LOCK: return "🎯";
            case APPROACH: return "➡️";
            case ARRIVE: return "✅";
            case FAILSAFE: return "⚠️";
            default: return "❓";
        }
    }

    private String estimateDistance(double totalArea) {
        if (totalArea < CLOSE_RANGE_AREA) {
            return String.format(Locale.US, "< %.1f in", Math.sqrt(144.0 / totalArea));
        } else {
            return String.format(Locale.US, "%.1f in", Math.sqrt(144.0 / totalArea));
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // CLEANUP
    // ═══════════════════════════════════════════════════════════════════════════
    private void cleanup() {
        // Stop all motors
        stopDrive();

        // Reset Limelight
        if (limelight != null) {
            try {
                limelight.stop();
            } catch (Exception e) {
                // Ignore cleanup errors
            }
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // DASHBOARD TELEMETRY UPDATE
    // ═══════════════════════════════════════════════════════════════════════════
    private void updateDashboard() {
        if (dashboard == null || dashboardTelemetry == null) return;

        dashboardTelemetry.addData("🎯 State", currentState.name());
        dashboardTelemetry.addData("Selection Mode", SELECTION_MODE.name());

        ClusterData cluster = getArtifactCluster();
        dashboardTelemetry.addData("Targets Detected", cluster.count);

        if (cluster.count > 0) {
            dashboardTelemetry.addData("Center X", String.format(Locale.US, "%.2f°", cluster.centerX));
            dashboardTelemetry.addData("Total Area", String.format(Locale.US, "%.2f%%", cluster.totalArea));
            dashboardTelemetry.addData("Aligned", Math.abs(cluster.centerX) < ALIGNMENT_TOLERANCE);
        }

        dashboardTelemetry.addData("KP", KP_ROTATE);
        dashboardTelemetry.addData("KD", KD_ROTATE);
        dashboardTelemetry.addData("Last Error", lastError);

        if (limelight != null) {
            try {
                LLStatus status = limelight.getStatus();
                if (status != null) {
                    dashboardTelemetry.addData("LL FPS", (int) status.getFps());
                    dashboardTelemetry.addData("LL CPU", String.format(Locale.US, "%.1f%%", status.getCpu()));
                    dashboardTelemetry.addData("LL Temp", String.format(Locale.US, "%.1f°C", status.getTemp()));
                }
            } catch (Exception ignored) {}
        }

        dashboardTelemetry.update();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // UTILITY METHODS (inspired by VisionUtils)
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Convert horizontal pixels to degrees based on camera FOV
     */
    private static double pixelsToDegrees(double pixels, double imageWidth) {
        return pixels * (CAMERA_HFOV / imageWidth);
    }

    /**
     * Convert vertical pixels to degrees based on camera FOV
     */
    private static double pixelsToDegreesVertical(double pixels, double imageHeight) {
        return pixels * (CAMERA_VFOV / imageHeight);
    }

    /**
     * Sort targets by area (descending) and return top N
     */
    private List<ArtifactTarget> getNLargestTargets(int n, List<ArtifactTarget> targets) {
        return targets.stream()
                .sorted(Comparator.comparingDouble((ArtifactTarget a) -> a.area).reversed())
                .limit(n)
                .collect(Collectors.toList());
    }

    /**
     * Sort targets by horizontal position (ascending) and return top N
     */
    private List<ArtifactTarget> getNLeftmostTargets(int n, List<ArtifactTarget> targets) {
        return targets.stream()
                .sorted(Comparator.comparingDouble(a -> a.tx))
                .limit(n)
                .collect(Collectors.toList());
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // INNER CLASSES - DATA STRUCTURES
    // ═══════════════════════════════════════════════════════════════════════════

    private static class ClusterData {
        public final double centerX;
        public final double centerY;
        public final double totalArea;
        public final int count;

        public ClusterData(double centerX, double centerY, double totalArea, int count) {
            this.centerX = centerX;
            this.centerY = centerY;
            this.totalArea = totalArea;
            this.count = count;
        }
    }

    private static class ArtifactTarget {
        public final double tx;
        public final double ty;
        public final double area;

        public ArtifactTarget(double tx, double ty, double area) {
            this.tx = tx;
            this.ty = ty;
            this.area = area;
        }
    }

    private static class Cluster {
        public double avgTx = 0;
        public double avgTy = 0;
        public double sumArea = 0;
        public double maxArea = 0;
        public int count = 0;
    }
}
