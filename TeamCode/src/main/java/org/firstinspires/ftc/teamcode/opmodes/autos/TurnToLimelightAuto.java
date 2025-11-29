package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

import java.util.List;

/**
 * Autonomous OpMode that CONTINUOUSLY tracks an AprilTag with the Limelight.
 *
 * BEHAVIOR:
 * 1. Spins in place until an AprilTag is detected
 * 2. Once found, continuously adjusts rotation to keep facing the tag
 * 3. Stops when aligned and holds position (or when timeout reached)
 *
 * UPDATED: Now uses REAL Limelight3A hardware and continuously tracks the target!
 *
 * CONFIGURATION:
 * - TARGET_APRILTAG_ID: Set to specific tag ID or -1 to track ANY AprilTag
 * - LIMELIGHT_PIPELINE: Pipeline number configured for AprilTag detection (usually 0)
 *
 * LIMELIGHT SETUP:
 * 1. Configure a Limelight pipeline for AprilTag detection (36h11 family for FTC)
 * 2. Ensure Limelight is connected via hardware config as "limelight"
 *
 * PID TUNING:
 * - kP: Increase for faster response, decrease if oscillating
 * - kD: Increase to reduce overshoot and oscillation
 */
@Autonomous(name = "Turn to AprilTag (Limelight)", group = "Auto")
public class TurnToLimelightAuto extends LinearOpMode {

    // ========================================================================================
    // CONFIGURATION
    // ========================================================================================
    private static final int TARGET_APRILTAG_ID = -1;  // -1 = track ANY tag, or set specific ID
    private static final int LIMELIGHT_PIPELINE = 0;

    // ========================================================================================
    // PID Constants
    // ========================================================================================
    private static final double kP = 0.02;    // Proportional gain - increase for faster response
    private static final double kD = 0.003;   // Derivative gain - increase to reduce oscillation

    // Power limits
    private static final double MAX_POWER = 0.5;      // Maximum rotation power
    private static final double MIN_POWER = 0.08;     // Minimum power to overcome friction
    private static final double SEARCH_POWER = 0.25;  // Power when searching for tag

    // Tolerances
    private static final double ALIGNMENT_TOLERANCE = 1.5;  // ±1.5° on TX to consider aligned
    private static final double SETTLE_TIME_MS = 500.0;     // Hold alignment for 500ms before success
    private static final double SEARCH_TIMEOUT_SECONDS = 10.0;  // Max time to search for tag
    private static final double TRACK_TIMEOUT_SECONDS = 15.0;   // Max time for entire operation

    // ========================================================================================
    // Hardware
    // ========================================================================================
    private MecanumDrive drive;
    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        drive = MecanumDrive.INSTANCE;

        // Initialize Limelight3A
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.start();
            limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
            telemetry.addData("Limelight", "✓ Connected");
        } catch (Exception e) {
            telemetry.addData("Limelight ERROR", e.getMessage());
            telemetry.update();
            sleep(5000);
            return;
        }

        // Initialize drive
        IMU imu = hardwareMap.get(IMU.class, "imu");
        drive.init(hardwareMap, imu);

        telemetry.addData("Status", "✓ Ready");
        telemetry.addData("Target Tag", TARGET_APRILTAG_ID == -1 ? "ANY" : TARGET_APRILTAG_ID);
        telemetry.addData("Mode", "Search → Track → Hold");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Phase 1: Search for AprilTag
            boolean found = searchForAprilTag();

            if (!found) {
                telemetry.addData("Status", "❌ FAILED - No tag found");
                telemetry.addData("Timeout", "%.1f seconds", SEARCH_TIMEOUT_SECONDS);
                telemetry.update();
                sleep(3000);
                return;
            }

            // Phase 2: Continuously track the AprilTag
            boolean success = trackAprilTag();

            // Report final results
            if (success) {
                telemetry.addData("Status", "✓ SUCCESS - Aligned!");
            } else {
                telemetry.addData("Status", "⚠ TIMEOUT - Could not maintain alignment");
            }

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                telemetry.addData("Final TX", "%.2f°", result.getTx());
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (fiducials != null && !fiducials.isEmpty()) {
                    telemetry.addData("Tag ID", fiducials.get(0).getFiducialId());
                }
            }

            telemetry.update();
            sleep(3000);
        }
    }

    /**
     * Phase 1: Spin in place until an AprilTag is detected
     * @return true if tag found, false if timeout
     */
    private boolean searchForAprilTag() {
        ElapsedTime searchTimer = new ElapsedTime();
        telemetry.addData("Phase", "🔍 SEARCHING for AprilTag...");
        telemetry.addData("Action", "Spinning to find tag");
        telemetry.update();

        searchTimer.reset();

        while (opModeIsActive() && searchTimer.seconds() < SEARCH_TIMEOUT_SECONDS) {
            LLResult result = limelight.getLatestResult();

            // Check if we found a valid target
            if (result != null && result.isValid()) {
                // Check if it's the right tag (if specific ID requested)
                if (TARGET_APRILTAG_ID == -1) {
                    // Any tag is acceptable
                    drive.setPowerRotation(0.0);
                    telemetry.addData("Phase", "✓ TAG FOUND!");
                    telemetry.update();
                    sleep(200);
                    return true;
                } else {
                    // Check if it's the target tag
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    if (fiducials != null && !fiducials.isEmpty()) {
                        long detectedId = fiducials.get(0).getFiducialId();
                        if (detectedId == TARGET_APRILTAG_ID) {
                            drive.setPowerRotation(0.0);
                            telemetry.addData("Phase", "✓ TARGET TAG FOUND!");
                            telemetry.addData("Tag ID", detectedId);
                            telemetry.update();
                            sleep(200);
                            return true;
                        }
                    }
                }
            }

            // Keep spinning to search
            drive.setPowerRotation(SEARCH_POWER);

            telemetry.addData("Phase", "🔍 SEARCHING...");
            telemetry.addData("Time", "%.1f / %.1f s", searchTimer.seconds(), SEARCH_TIMEOUT_SECONDS);
            telemetry.addData("Target", TARGET_APRILTAG_ID == -1 ? "ANY tag" : "Tag ID " + TARGET_APRILTAG_ID);
            telemetry.update();

            sleep(20);
        }

        // Timeout - stop spinning
        drive.setPowerRotation(0.0);
        return false;
    }

    /**
     * Phase 2: Continuously track the AprilTag, adjusting rotation to keep facing it
     * @return true if successfully held alignment, false if timeout or lost tag
     */
    private boolean trackAprilTag() {
        ElapsedTime trackTimer = new ElapsedTime();
        ElapsedTime settleTimer = new ElapsedTime();
        boolean inTolerance = false;
        double previousError = 0.0;

        telemetry.addData("Phase", "🎯 TRACKING AprilTag...");
        telemetry.update();

        trackTimer.reset();

        while (opModeIsActive() && trackTimer.seconds() < TRACK_TIMEOUT_SECONDS) {
            // Get REAL-TIME Limelight data
            LLResult result = limelight.getLatestResult();

            if (result == null || !result.isValid()) {
                telemetry.addData("Phase", "⚠ Lost target");
                telemetry.update();
                drive.setPowerRotation(0.0);
                sleep(100);
                continue;  // Wait for tag to reappear
            }

            // Get horizontal offset (TX) - this is our error
            double tx = result.getTx();
            double error = tx;  // Positive = target is right, negative = target is left

            // Calculate derivative (rate of change)
            double derivative = (error - previousError) / 0.02;  // Approx loop time

            // PD control (no integral to avoid windup)
            double power = kP * error + kD * derivative;

            // Apply power limits
            if (Math.abs(power) > MAX_POWER) {
                power = Math.copySign(MAX_POWER, power);
            } else if (Math.abs(power) < MIN_POWER && Math.abs(error) > ALIGNMENT_TOLERANCE) {
                // Apply minimum power if not aligned
                power = Math.copySign(MIN_POWER, power);
            } else if (Math.abs(error) <= ALIGNMENT_TOLERANCE) {
                // Within tolerance - use very small power for fine adjustment
                power = 0.0;
            }

            // Check if we're aligned
            boolean currentlyAligned = Math.abs(tx) <= ALIGNMENT_TOLERANCE;

            if (currentlyAligned) {
                if (!inTolerance) {
                    settleTimer.reset();
                    inTolerance = true;
                }

                // Check if we've been aligned long enough
                if (settleTimer.milliseconds() > SETTLE_TIME_MS) {
                    drive.setPowerRotation(0.0);
                    telemetry.addData("Phase", "✓ LOCKED ON TARGET!");
                    telemetry.update();
                    return true;  // Success!
                }
            } else {
                inTolerance = false;
            }

            // Apply rotation
            drive.setPowerRotation(power);

            // Update telemetry
            telemetry.addData("Phase", "🎯 TRACKING");
            telemetry.addData("TX (offset)", String.format("%.2f°", tx));
            telemetry.addData("Power", String.format("%.3f", power));
            telemetry.addData("Aligned", currentlyAligned ? "✓ YES" : "✗ NO");

            if (inTolerance) {
                telemetry.addData("Hold Timer", String.format("%.0f / %.0f ms",
                    settleTimer.milliseconds(), SETTLE_TIME_MS));
            }

            // Show tag ID if available
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                telemetry.addData("Tag ID", fiducials.get(0).getFiducialId());
            }

            telemetry.addData("Runtime", String.format("%.1f s", trackTimer.seconds()));
            telemetry.update();

            previousError = error;
            sleep(20);
        }

        // Timeout
        drive.setPowerRotation(0.0);
        return false;
    }
}
