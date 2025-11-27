package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

/**
 * Autonomous OpMode that rotates the robot in place until it faces a specific AprilTag target detected by Limelight.
 *
 * CONFIGURATION:
 * Change TARGET_APRILTAG_ID below to match the AprilTag you want to track.
 * Set to -1 to track any visible AprilTag.
 *
 * LIMELIGHT SETUP:
 * 1. Configure a Limelight pipeline for AprilTag detection (36h11 family for FTC)
 * 2. Set the pipeline number in LIMELIGHT_PIPELINE if not using pipeline 0
 * 3. Ensure Limelight is connected and has correct IP address in Limelight.java
 *
 * ALGORITHM:
 * 1. Configure Limelight to track specific AprilTag ID
 * 2. Read current heading from IMU
 * 3. Read target offset (yaw) from Limelight (only if correct tag is visible)
 * 4. Calculate target heading = current heading + limelight yaw
 * 5. Run PID loop to minimize error between target and current heading
 * 6. Stop when within tolerance for sufficient settle time
 *
 * PID TUNING GUIDE:
 * - kP: Start at 0.015. If oscillates, reduce. If too slow, increase.
 * - kI: Keep at 0.0 unless you have steady-state error (robot never quite reaches target).
 * - kD: Start at 0.001. Increase to reduce overshoot.
 * - maxPower: Limit maximum rotation speed (0.4 = 40% power)
 * - minPower: Minimum power to overcome static friction (0.07 = 7% power)
 *
 * MOTOR DIRECTION TROUBLESHOOTING:
 * If robot rotates in WRONG direction (e.g., turns right when it should turn left):
 * Option 1: In MecanumDrive.setPowerRotation(), swap the signs on power
 * Option 2: In MecanumDrive constructor, flip which motors are .reversed()
 *
 * TOLERANCE SETTINGS:
 * - limelightTolerance: How close to target center (±1.5° default)
 * - headingTolerance: How close to final heading (±2.0° default)
 * - settleTime: How long to stay in tolerance before stopping (250ms default)
 *
 * Based on FTC SDK samples: ConceptAprilTag.java, ConceptAprilTagLocalization.java
 */
@Autonomous(name = "Turn to AprilTag (Limelight)", group = "Auto")
public class TurnToLimelightAuto extends LinearOpMode {

    // ========================================================================================
    // CONFIGURATION - CHANGE THESE VALUES
    // ========================================================================================

    /**
     * Target AprilTag ID to track.
     * Examples:
     * - Set to 5 to track AprilTag ID 5
     * - Set to 1 to track AprilTag ID 1
     * - Set to -1 to track ANY visible AprilTag
     *
     * Common FTC AprilTag IDs for DECODE (2025-2026):
     * - Blue Alliance: IDs 1, 2, 3
     * - Red Alliance: IDs 4, 5, 6
     * - Obelisk tags: IDs 7-10 (typically not used for localization)
     */
    private static final int TARGET_APRILTAG_ID = 20;  // CHANGE THIS to your target tag

    /**
     * Limelight pipeline configured for AprilTag detection.
     * Set this to match your Limelight's AprilTag pipeline number (0-9).
     */
    private static final int LIMELIGHT_PIPELINE = 0;  // CHANGE THIS if needed

    // ========================================================================================
    // PID Constants - TUNE THESE for your robot
    // ========================================================================================
    private static final double kP = 0.015;  // Proportional gain - adjust if too slow or oscillates
    private static final double kI = 0.0;    // Integral gain - usually keep at 0
    private static final double kD = 0.001;  // Derivative gain - increase to reduce overshoot

    // Power limits
    private static final double MAX_POWER = 0.4;   // Maximum rotation power (40%)
    private static final double MIN_POWER = 0.07;  // Minimum power to overcome friction (7%)

    // Tolerance and timing
    private static final double LIMELIGHT_TOLERANCE = 1.5;  // ±1.5° on Limelight yaw
    private static final double HEADING_TOLERANCE = 2.0;    // ±2.0° on heading error
    private static final double SETTLE_TIME_MS = 250.0;     // Must be in tolerance for 250ms
    private static final double TIMEOUT_SECONDS = 6.0;      // Abort after 6 seconds

    // ========================================================================================
    // Subsystems
    // ========================================================================================
    private MecanumDrive drive;
    private Limelight limelight;

    @Override
    public void runOpMode() {
        // Initialize hardware
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        drive = MecanumDrive.INSTANCE;
        limelight = Limelight.INSTANCE;

        // Get IMU from hardwareMap and initialize drive
        IMU imu = hardwareMap.get(IMU.class, "imu");
        drive.init(hardwareMap, imu);

        // Configure Limelight for AprilTag tracking
        limelight.setTargetAprilTagId(TARGET_APRILTAG_ID);
        limelight.setPipeline(LIMELIGHT_PIPELINE);

        String targetInfo = TARGET_APRILTAG_ID == 20 ?
            "Any AprilTag" :
            "AprilTag ID " + TARGET_APRILTAG_ID;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Target AprilTag ID", TARGET_APRILTAG_ID == 20 ? "ANY" : TARGET_APRILTAG_ID);
        telemetry.addData("Limelight Pipeline", LIMELIGHT_PIPELINE);
        telemetry.addData("Info", "Robot will rotate to face target");
        telemetry.addData("Tuning", "kP=%.3f, kD=%.3f", kP, kD);
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Check if target is initially visible
            if (!limelight.hasTarget()) {
                String targetDesc = TARGET_APRILTAG_ID == 20 ?
                    "any AprilTag" :
                    "AprilTag ID " + TARGET_APRILTAG_ID;

                telemetry.addData("Error", "Target not detected!");
                telemetry.addData("Looking for", targetDesc);
                telemetry.addData("Action", "Place target in view or adjust");

                int detectedId = limelight.getDetectedTagId();
                if (detectedId != 20) {
                    telemetry.addData("Detected Tag ID", detectedId);
                    telemetry.addData("Note", "This is not the target tag");
                }

                telemetry.update();
                sleep(3000);
                return;
            }

            // Execute turn
            boolean success = turnToAprilTag();

            // Report results
            if (success) {
                telemetry.addData("Status", "SUCCESS - Target acquired!");
            } else {
                telemetry.addData("Status", "TIMEOUT - Could not align");
            }
            telemetry.addData("Target Tag ID", TARGET_APRILTAG_ID == 20 ? "ANY" : TARGET_APRILTAG_ID);
            telemetry.addData("Detected Tag ID", limelight.getDetectedTagId());
            telemetry.addData("Final Heading", "%.2f°", drive.getHeading());
            telemetry.addData("Final Limelight Yaw", "%.2f°", limelight.getYaw());
            telemetry.update();

            sleep(2000);
        }
    }

    /**
     * Main control loop: rotate robot until facing the AprilTag target.
     *
     * @return true if successfully aligned, false if timeout
     */
    private boolean turnToAprilTag() {
        ElapsedTime runtime = new ElapsedTime();
        ElapsedTime settleTimer = new ElapsedTime();
        boolean inTolerance = false;

        // PID state variables
        double previousError = 0.0;
        double integral = 0.0;

        // Get initial heading and calculate target
        double initialHeading = drive.getHeading();
        double limelightYaw = limelight.getYaw();
        double targetHeading = normalizeAngle(initialHeading + limelightYaw);

        telemetry.addData("Initial Heading", "%.2f°", initialHeading);
        telemetry.addData("Limelight Offset", "%.2f°", limelightYaw);
        telemetry.addData("Target Heading", "%.2f°", targetHeading);
        telemetry.addData("Target Tag ID", TARGET_APRILTAG_ID == -1 ? "ANY" : TARGET_APRILTAG_ID);
        telemetry.addData("Detected Tag ID", limelight.getDetectedTagId());
        telemetry.update();

        runtime.reset();

        while (opModeIsActive() && runtime.seconds() < TIMEOUT_SECONDS) {
            // Check if we lost the target
            if (!limelight.hasTarget()) {
                String targetDesc = TARGET_APRILTAG_ID == 20 ?
                    "any AprilTag" :
                    "AprilTag ID " + TARGET_APRILTAG_ID;

                telemetry.addData("Warning", "Target lost!");
                telemetry.addData("Looking for", targetDesc);

                int detectedId = limelight.getDetectedTagId();
                if (detectedId != 20) {
                    telemetry.addData("Detected Tag ID", detectedId);
                }

                telemetry.update();
                // Slow rotation to search
                drive.setPowerRotation(0.1);
                sleep(50);
                continue;
            }

            // Read current state
            double currentHeading = drive.getHeading();
            double currentLimelightYaw = limelight.getYaw();

            // Update target heading based on fresh Limelight data
            // This allows tracking a moving target
            targetHeading = normalizeAngle(currentHeading + currentLimelightYaw);

            // Calculate error
            double error = normalizeAngle(targetHeading - currentHeading);

            // PID calculations
            double derivative = error - previousError;
            integral += error;

            // Anti-windup: limit integral accumulation
            integral = Math.max(-10, Math.min(10, integral));

            // Calculate output
            double output = kP * error + kI * integral + kD * derivative;

            // Apply power limits and minimum power threshold
            output = applyPowerLimits(output);

            // Send power to motors
            drive.setPowerRotation(output);

            // Check if within tolerance
            boolean limelightOK = Math.abs(currentLimelightYaw) < LIMELIGHT_TOLERANCE;
            boolean headingOK = Math.abs(error) < HEADING_TOLERANCE;

            if (limelightOK && headingOK) {
                if (!inTolerance) {
                    inTolerance = true;
                    settleTimer.reset();
                }

                // Check if settled for required duration
                if (settleTimer.milliseconds() > SETTLE_TIME_MS) {
                    drive.stop();
                    return true; // Success!
                }
            } else {
                inTolerance = false;
            }

            // Update telemetry
            telemetry.addData("Status", "Aligning to target...");
            telemetry.addData("Current Heading", "%.2f°", currentHeading);
            telemetry.addData("Target Heading", "%.2f°", targetHeading);
            telemetry.addData("Heading Error", "%.2f°", error);
            telemetry.addData("Limelight Yaw", "%.2f°", currentLimelightYaw);
            telemetry.addData("PID Output", "%.3f", output);
            telemetry.addData("Elapsed Time", "%.2f s", runtime.seconds());
            telemetry.addData("In Tolerance", inTolerance ? "YES" : "NO");
            if (inTolerance) {
                telemetry.addData("Settle Progress", "%.0f / %.0f ms",
                    settleTimer.milliseconds(), SETTLE_TIME_MS);
            }
            telemetry.update();

            // Save state for next iteration
            previousError = error;

            // Small sleep to avoid CPU thrashing
            sleep(20);

            // Check for stop request
            if (isStopRequested()) {
                break;
            }
        }

        // Timeout or stop requested
        drive.stop();
        return false;
    }

    /**
     * Apply power limits and minimum threshold to overcome static friction.
     *
     * @param power raw PID output
     * @return limited power command
     */
    private double applyPowerLimits(double power) {
        // Clip to maximum
        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

        // Apply minimum power threshold (deadband compensation)
        if (Math.abs(power) > 0.001) {  // Avoid sign errors at exactly 0
            if (power > 0) {
                power = Math.max(power, MIN_POWER);
            } else {
                power = Math.min(power, -MIN_POWER);
            }
        }

        return power;
    }

    /**
     * Normalize an angle to the range [-180, 180] degrees.
     * This ensures proper error calculation across the ±180° discontinuity.
     *
     * @param angle input angle in degrees
     * @return normalized angle in range [-180, 180]
     */
    private double normalizeAngle(double angle) {
        while (angle > 90) angle -= 180;
        while (angle < -90) angle += 180;
        return angle;
    }
}
