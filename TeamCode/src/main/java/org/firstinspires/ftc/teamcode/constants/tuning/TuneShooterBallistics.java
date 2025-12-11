package org.firstinspires.ftc.teamcode.constants.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.LimelightConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterBallistics;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;

import java.util.List;

/**
 * TuneShooterBallistics - Distance-Based Shooter Tuning
 *
 * This OpMode helps you tune the ballistics model (RPM & Hood vs Distance).
 * It uses Limelight to measure distance, then applies the ballistics formulas.
 *
 * ==================== HOW TO TUNE ====================
 *
 * PREREQUISITES:
 * 1. First tune PIDF using TuneShooter (so shooter can hold target RPM)
 * 2. Tune LimelightConstants so distance readings are accurate
 *
 * TUNING MODES:
 *
 * MODE 1: MANUAL OVERRIDE (manualOverride = true)
 *   - Manually set manualRpm and manualHood from Dashboard
 *   - Shoot at known distances and record what RPM/Hood works
 *   - Build a table of Distance -> RPM, Hood values
 *
 * MODE 2: AUTO BALLISTICS (manualOverride = false)
 *   - Uses ShooterBallistics formulas to compute RPM/Hood from distance
 *   - Adjust RPM_A, RPM_B, RPM_C and HOOD_A, HOOD_B to match your data
 *
 * STEP-BY-STEP:
 * 1. Set manualOverride = true
 * 2. Place robot at 1m, adjust manualRpm/manualHood until shots land
 * 3. Record values. Repeat at 2m, 3m, etc.
 * 4. Set manualOverride = false
 * 5. Adjust ShooterBallistics coefficients to match your recorded data
 * 6. Test at all distances - fine tune as needed
 *
 * CONTROLS:
 *   B = Start shooter
 *   X = Stop shooter
 *   A = Reset PIDF state
 *   Right Bumper = Hold to shoot (alternative)
 *
 * ==================================================================
 */
@Config
@TeleOp(name = "TuneShooterBallistics", group = "Tuning")
public class TuneShooterBallistics extends OpMode {

    // ===== DASHBOARD TUNABLE - TUNING MODE =====
    public static boolean shooterEnabled = false;
    public static boolean manualOverride = false;  // true = use manual values, false = use ballistics
    public static double manualRpm = 3000.0;       // Manual RPM override
    public static double manualHood = 0.5;         // Manual hood override

    // ===== DASHBOARD TUNABLE - DISTANCE OVERRIDE (for testing without Limelight) =====
    public static boolean useManualDistance = false;
    public static double manualDistance = 2.0;     // Manual distance in meters

    // Hardware
    private DcMotorEx shooterRight;
    private DcMotorEx shooterLeft;
    private Servo hoodServo;
    private Limelight3A limelight;
    private List<LynxModule> allHubs;

    // PIDF state
    private double integral = 0.0;
    private double lastError = 0.0;

    // Measured values
    private double currentDistance = 0.0;
    private double targetRpm = 0.0;
    private double targetHood = 0.0;
    private double currentRpm = 0.0;
    private int visibleTags = 0;
    private int chosenTagId = -1;

    // Dashboard
    private FtcDashboard dashboard;

    @Override
    public void init() {
        // Enable bulk reads for fresh sensor data every loop
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Get shooter hardware directly
        shooterRight = hardwareMap.get(DcMotorEx.class, "ShooterRight");
        shooterLeft = hardwareMap.get(DcMotorEx.class, "ShooterLeft");
        hoodServo = hardwareMap.get(Servo.class, "HoodServo");

        // Configure motors
        shooterRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooterLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooterLeft.setDirection(DcMotorEx.Direction.REVERSE);

        // Use RUN_WITHOUT_ENCODER for raw power control
        // getVelocity() still works in this mode!
        shooterRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize Limelight
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(0);
            limelight.start();
        } catch (Exception e) {
            limelight = null;
        }

        // Setup dashboard
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        if (limelight != null) {
            dashboard.startCameraStream(limelight, 0);
        }

        // Set hood to default
        hoodServo.setPosition(ShooterConstants.defaultPos);

        telemetry.addLine("=== TuneShooterBallistics Ready ===");
        telemetry.addLine("B = Start | X = Stop | A = Reset PIDF");
        telemetry.addLine("Set manualOverride=true to manually set RPM/Hood");
        telemetry.update();
    }

    @Override
    public void loop() {
        // ===== CONTROLS =====
        if (gamepad1.b || gamepad1.right_bumper) {
            shooterEnabled = true;
        }
        if (gamepad1.x) {
            shooterEnabled = false;
            integral = 0;
            lastError = 0;
        }
        if (gamepad1.a) {
            integral = 0;
            lastError = 0;
        }

        // ===== 1. GET DISTANCE =====
        currentDistance = 0;
        visibleTags = 0;
        chosenTagId = -1;

        if (useManualDistance) {
            currentDistance = manualDistance;
        } else if (limelight != null) {
            try {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    if (fiducials != null && !fiducials.isEmpty()) {
                        visibleTags = fiducials.size();

                        // Find closest tag (largest area)
                        LLResultTypes.FiducialResult bestTag = fiducials.get(0);
                        double bestArea = 0;
                        for (LLResultTypes.FiducialResult tag : fiducials) {
                            double area = Math.abs(tag.getTargetXPixels() * tag.getTargetYPixels());
                            if (area > bestArea) {
                                bestArea = area;
                                bestTag = tag;
                            }
                        }
                        chosenTagId = bestTag.getFiducialId();

                        // Calculate distance using trigonometry
                        double ty = result.getTy();
                        double camPitchRad = Math.toRadians(LimelightConstants.CAM_PITCH_DEG);
                        double angleRad = camPitchRad + Math.toRadians(ty);

                        if (Math.abs(Math.tan(angleRad)) > 0.001) {
                            double dist = (LimelightConstants.TAG_HEIGHT_M - LimelightConstants.CAM_HEIGHT_M) / Math.tan(angleRad);
                            dist *= LimelightConstants.DISTANCE_SCALE;
                            currentDistance = Math.max(LimelightConstants.MIN_DISTANCE_M,
                                    Math.min(LimelightConstants.MAX_DISTANCE_M, dist));
                        }
                    }
                }
            } catch (Exception e) {
                // Limelight error - keep distance at 0
            }
        }

        // ===== 2. CALCULATE TARGET RPM & HOOD =====
        if (manualOverride) {
            targetRpm = manualRpm;
            targetHood = manualHood;
        } else if (currentDistance > 0) {
            // Use ballistics model
            targetRpm = ShooterBallistics.RPM_A
                    + ShooterBallistics.RPM_B * currentDistance
                    + ShooterBallistics.RPM_C * currentDistance * currentDistance;
            targetRpm = Math.max(ShooterBallistics.MIN_RPM, Math.min(ShooterBallistics.MAX_RPM, targetRpm));

            targetHood = ShooterBallistics.HOOD_A + ShooterBallistics.HOOD_B * currentDistance;
            targetHood = Math.max(ShooterBallistics.MIN_HOOD, Math.min(ShooterBallistics.MAX_HOOD, targetHood));
        } else {
            // No valid distance - use defaults
            targetRpm = ShooterBallistics.MIN_RPM;
            targetHood = ShooterBallistics.HOOD_A;
        }

        // ===== 3. APPLY HOOD =====
        hoodServo.setPosition(targetHood);

        // ===== 4. GET CURRENT RPM =====
        double ticksPerSecond = shooterRight.getVelocity();
        currentRpm = (ticksPerSecond / 112.0) * 60.0 * 5;  // Same multiplier as TeleOp

        // ===== 5. RUN SHOOTER WITH PIDF =====
        double output = 0;
        double error = 0;

        if (shooterEnabled && targetRpm > 0) {
            error = targetRpm - currentRpm;
            integral += error;
            double derivative = error - lastError;
            lastError = error;

            // PIDF calculation using ShooterConstants
            output = ShooterConstants.kF * targetRpm
                    + ShooterConstants.kP * error
                    + ShooterConstants.kI * integral
                    + ShooterConstants.kD * derivative;

            output = Math.max(ShooterConstants.MIN_POWER, Math.min(ShooterConstants.MAX_POWER, output));

            shooterRight.setPower(output);
            shooterLeft.setPower(output);
        } else {
            shooterRight.setPower(0);
            shooterLeft.setPower(0);
            integral = 0;
            lastError = 0;
        }

        // ===== 6. SEND TO DASHBOARD GRAPH =====
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Distance (m)", currentDistance);
        packet.put("Target RPM", targetRpm);
        packet.put("Current RPM", currentRpm);
        packet.put("RPM Error", error);
        packet.put("Hood Position", targetHood);
        packet.put("Output Power", output);
        dashboard.sendTelemetryPacket(packet);

        // ===== 7. DRIVER STATION TELEMETRY =====
        telemetry.addLine("===== SHOOTER BALLISTICS TUNING =====");
        telemetry.addData("Status", shooterEnabled ? "✓ RUNNING" : "✗ STOPPED");
        telemetry.addData("Mode", manualOverride ? "MANUAL OVERRIDE" : "AUTO BALLISTICS");

        telemetry.addLine();
        telemetry.addLine("----- DISTANCE -----");
        telemetry.addData("Source", useManualDistance ? "Manual" : (limelight != null ? "Limelight" : "N/A"));
        telemetry.addData("Tags Visible", visibleTags);
        telemetry.addData("Tag ID", chosenTagId);

        telemetry.addLine();
        telemetry.addLine("----- SHOOTER -----");
        telemetry.addData("Target RPM", "%.0f", targetRpm);
        telemetry.addData("Current RPM", "%.0f", currentRpm);
        telemetry.addData("Error", "%.0f", error);
        telemetry.addData("Output Power", "%.3f", output);

        telemetry.addLine();
        telemetry.addLine("----- HOOD -----");
        telemetry.addData("Target Hood", "%.3f", targetHood);

        telemetry.addLine();
        telemetry.addLine("----- BALLISTICS MODEL (ShooterBallistics) -----");
        telemetry.addData("RPM = A + B*d + C*d²", "");
        telemetry.addData("  RPM_A (base)", "%.0f", ShooterBallistics.RPM_A);
        telemetry.addData("  RPM_B (linear)", "%.0f", ShooterBallistics.RPM_B);
        telemetry.addData("  RPM_C (quadratic)", "%.0f", ShooterBallistics.RPM_C);
        telemetry.addData("Hood = A + B*d", "");
        telemetry.addData("  HOOD_A", "%.3f", ShooterBallistics.HOOD_A);
        telemetry.addData("  HOOD_B", "%.3f", ShooterBallistics.HOOD_B);

        telemetry.addLine();
        telemetry.addLine("----- CONTROLS -----");
        telemetry.addData("B / RB", "Start Shooter");
        telemetry.addData("X", "Stop Shooter");
        telemetry.addData("A", "Reset PIDF");

        telemetry.update();
    }

    @Override
    public void stop() {
        shooterRight.setPower(0);
        shooterLeft.setPower(0);
        if (limelight != null) {
            limelight.stop();
        }
    }
}

