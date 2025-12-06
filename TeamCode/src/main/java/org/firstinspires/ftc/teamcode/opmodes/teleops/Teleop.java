package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.IntakeSeqCmd;
import org.firstinspires.ftc.teamcode.commands.ShooterOffCmd;
import org.firstinspires.ftc.teamcode.commands.ShooterOnCmd;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

/**
 * Main TeleOp with AUTOMATIC TURRET TRACKING!
 *
 * Driver controls the robot movement normally, but the turret automatically
 * tracks the CLOSEST AprilTag detected by the Limelight.
 *
 * Features:
 * - Full mecanum drive control
 * - Automatic turret tracking (tracks ANY AprilTag)
 * - Selects closest tag when multiple are visible
 * - All normal shooter/intake controls
 * - Real-time telemetry on dashboard
 * - Camera stream visible on FTC Dashboard
 * - Toggle tracking on/off with X button
 */
@Config
@TeleOp(name = "TeleOp")
public class Teleop extends NextFTCOpMode {
    // Tunable via FTC Dashboard
    public static double TRACKING_GAIN = 0.08;  // Reduced from 0.15 - much smoother
    public static double SMOOTHING = 0.7;  // Exponential smoothing (0.0-1.0, lower = smoother)
    public static double TURRET_LIMIT_DEG = 90.0;  // Max turret rotation
    public static double DEADBAND = 3.0;  // Increased from 2.0 - larger tolerance to prevent jitter
    public static boolean AUTO_TRACK_ENABLED = true;  // Enable/disable tracking
    public static double NO_TARGET_TIMEOUT_SEC = 5.0;  // Time before returning to center when no target detected

    public Teleop() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Turret.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private final MotorEx frontLeftMotor = new MotorEx("frontLeftMotor").brakeMode().reversed();
    private final MotorEx frontRightMotor = new MotorEx("frontRightMotor").brakeMode();
    private final MotorEx backLeftMotor = new MotorEx("backLeftMotor").brakeMode().reversed();
    private final MotorEx backRightMotor = new MotorEx("backRightMotor").brakeMode();

    Limelight3A limelight;
    double motorTargetX = 0.0;
    double smoothedTx = 0.0;  // Smoothed TX value to prevent jitter
    boolean hasSeenTarget = false;  // Track if we've ever seen a target
    long lastTargetSeenTime = 0;  // Timestamp of last valid target detection

    double servoPos = ShooterConstants.defaultPos;
    double shooterPower = 0.5;

    // Shooter timing
    boolean hasRumbled = false;
    private long shooterStartTime = 0;
    private boolean shooterTiming = false;

    @Override
    public void onInit() {
        // Setup dashboard and camera stream
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        Intake.INSTANCE.defaultPos.schedule();
        Shooter.INSTANCE.defaultPos.schedule();

        // IMPORTANT: Zero the turret and set initial target to 0 (front)
        Turret.INSTANCE.turret.zeroed();
        motorTargetX = 0.0;
        smoothedTx = 0.0;
        hasSeenTarget = false;
        lastTargetSeenTime = System.currentTimeMillis();  // Initialize timeout timer

        // Initialize Limelight with camera streaming
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.start();
            limelight.pipelineSwitch(0);

            // Stream camera to dashboard
            dashboard.startCameraStream(limelight, 0);

            telemetry.addData("Limelight", "✓ Connected");
            telemetry.addData("Camera", "✓ Streaming to dashboard");
        } catch (Exception e) {
            telemetry.addData("Limelight", "✗ ERROR: " + e.getMessage());
        }

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Mode", "Driver Control + Auto Turret");
        telemetry.addData("Turret", "Centered at 0° (FRONT)");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // Driver controls - normal mecanum drive
        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX().negate()
        );
        driverControlled.schedule();

        // Left Bumper - Intake sequence (press to start, release to stop)
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(() -> {
            IntakeSeqCmd.create().schedule();
        });
        Gamepads.gamepad1().leftBumper().whenBecomesFalse(() -> {
            ShooterOffCmd.create().schedule();
        });

        // Right Bumper - Shooter on
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> {
            shooterStartTime = System.currentTimeMillis();
            shooterTiming = true;
            ShooterOnCmd.create(shooterPower).schedule();
        });

        // A Button - Manual intake (hold to run, release to stop)
        Gamepads.gamepad1().a().whenBecomesTrue(() -> {
            Intake.INSTANCE.turnOn.schedule();
        });
        Gamepads.gamepad1().a().whenBecomesFalse(() -> {
            Intake.INSTANCE.zeroPower.schedule();
        });

        // B Button - Shooter off
        Gamepads.gamepad1().b().whenBecomesTrue(() -> {
            Shooter.INSTANCE.zeroPower.schedule();
        });

        // Shooter power adjustment
        Gamepads.gamepad1().dpadRight().whenBecomesTrue(() -> {
            shooterPower = Math.min(1.0, shooterPower + 0.1);
        });

        Gamepads.gamepad1().dpadLeft().whenBecomesTrue(() -> {
            shooterPower = Math.max(0.0, shooterPower - 0.1);
        });


        // Servo position adjustment
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(() -> {
            servoPos = Math.min(1.0, servoPos - 0.1);
            Shooter.INSTANCE.moveServo(servoPos).schedule();
        });

        Gamepads.gamepad1().dpadDown().whenBecomesTrue(() -> {
            servoPos = Math.max(0.0, servoPos + 0.1);
            Shooter.INSTANCE.moveServo(servoPos).schedule();
        });

        // Toggle auto-tracking with X button
        Gamepads.gamepad1().x().whenBecomesTrue(() -> {
            AUTO_TRACK_ENABLED = !AUTO_TRACK_ENABLED;
            if (!AUTO_TRACK_ENABLED) {
                motorTargetX = 0.0;  // Return to center when disabled
                hasSeenTarget = false;
            }
        });
    }

    @Override
    public void onUpdate() {
        // Simple shooter monitoring - just track power and basic RPM for display
        double rpm = Shooter.INSTANCE.getRPM() * 5;

        // ========================================================================
        // AUTOMATIC TURRET TRACKING - Tracks CLOSEST AprilTag
        // ========================================================================
        LLResult result = null;
        if (limelight != null) {
            try {
                result = limelight.getLatestResult();
            } catch (Exception e) {
                telemetry.addData("Limelight Error", e.getMessage());
            }
        }

        if (AUTO_TRACK_ENABLED && result != null && result.isValid()) {
            // Get all detected AprilTags
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (fiducials != null && !fiducials.isEmpty()) {
                // Find the CLOSEST tag (largest area = closest)
                LLResultTypes.FiducialResult closestTag = null;
                double largestArea = 0.0;

                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    // Calculate area from target XY coordinates (larger = closer)
                    double targetArea = fiducial.getTargetXPixels() * fiducial.getTargetYPixels();

                    if (closestTag == null || targetArea > largestArea) {
                        closestTag = fiducial;
                        largestArea = targetArea;
                    }
                }

                if (closestTag != null) {
                    hasSeenTarget = true;
                    lastTargetSeenTime = System.currentTimeMillis();  // Update last seen time

                    // Use TX from the result for turret control
                    double tx = result.getTx();

                    // FIXED: Apply exponential smoothing to prevent jitter
                    smoothedTx = SMOOTHING * smoothedTx + (1.0 - SMOOTHING) * tx;

                    // Apply deadband to prevent jitter when close to aligned
                    if (Math.abs(smoothedTx) > DEADBAND) {
                        // FIXED: Use proportional scaling - smaller adjustments when close to target
                        // This prevents overshooting and oscillation
                        double adjustment = smoothedTx * TRACKING_GAIN;

                        // Scale down adjustment even more when we're getting close
                        if (Math.abs(smoothedTx) < 10.0) {
                            adjustment *= 0.5;  // Half speed when within 10 degrees
                        }

                        motorTargetX += adjustment;

                        // Clamp to limits
                        motorTargetX = Math.max(-TURRET_LIMIT_DEG, Math.min(TURRET_LIMIT_DEG, motorTargetX));
                    }
                    // If within deadband, keep current target (don't update)

                    telemetry.addData("═══ TURRET TRACKING ═══", "");
                    telemetry.addData("Status", "✓ TRACKING");
                    telemetry.addData("Tag ID", closestTag.getFiducialId());
                    telemetry.addData("TX Offset (raw)", String.format("%.2f°", tx));
                    telemetry.addData("TX Offset (smooth)", String.format("%.2f°", smoothedTx));
                    telemetry.addData("Turret Angle", String.format("%.2f°", motorTargetX));
                    telemetry.addData("Target Area", String.format("%.2f%%", result.getTa()));
                    telemetry.addData("Tags Visible", fiducials.size());
                    telemetry.addData("Aligned", Math.abs(smoothedTx) <= DEADBAND ? "✓ YES" : "✗ NO");
                }
            } else {
                telemetry.addData("═══ TURRET TRACKING ═══", "");
                telemetry.addData("Status", "✗ No AprilTags detected");
                // Keep last known position if we've seen a target before
                if (!hasSeenTarget) {
                    motorTargetX = 0.0;  // Return to center if never seen target
                    smoothedTx = 0.0;
                }
            }
        } else if (!AUTO_TRACK_ENABLED) {
            telemetry.addData("═══ TURRET TRACKING ═══", "");
            telemetry.addData("Status", "⚠ DISABLED (Press X to enable)");
            telemetry.addData("Turret Angle", String.format("%.2f°", motorTargetX));
            smoothedTx = 0.0;  // Reset smoothing when disabled
        } else {
            telemetry.addData("═══ TURRET TRACKING ═══", "");
            telemetry.addData("Status", "✗ No valid target");
            telemetry.addData("Turret Angle", String.format("%.2f°", motorTargetX));
            // Keep last position or return to center if never tracked
            if (!hasSeenTarget) {
                motorTargetX = 0.0;
                smoothedTx = 0.0;
            }
        }

        // Check for timeout - return turret to center if no target detected for a while
        if (System.currentTimeMillis() - lastTargetSeenTime > NO_TARGET_TIMEOUT_SEC * 1000) {
            motorTargetX = 0.0;  // Return to center (front)
            smoothedTx = 0.0;
            hasSeenTarget = false;
        }

        // Apply turret target - use direct control instead of scheduling commands
        Turret.INSTANCE.setTargetDegrees(motorTargetX);

        // ========================================================================
        // SHOOTER TELEMETRY
        // ========================================================================
        telemetry.addData("", "");
        telemetry.addData("═══ SHOOTER ═══", "");
        telemetry.addData("RPM", String.format("%.0f", rpm));
        telemetry.addData("Power", String.format("%.1f", shooterPower));
        telemetry.addData("Servo Position", String.format("%.2f", servoPos));


        // ========================================================================
        // CONTROLS REMINDER
        // ========================================================================
        telemetry.addData("", "");
        telemetry.addData("═══ CONTROLS ═══", "");
        telemetry.addData("Left Stick", "Drive");
        telemetry.addData("Right Stick", "Rotate");
        telemetry.addData("X Button", "Toggle Turret Tracking");
        telemetry.addData("Left Bumper", "Intake Sequence");
        telemetry.addData("Right Bumper", "Shooter On");
        telemetry.addData("A Button", "Manual Intake");
        telemetry.addData("B Button", "Shooter Off");
        telemetry.addData("DPad Up/Down", "Adjust Servo Position");
        telemetry.addData("DPad Left/Right", "Adjust Shooter Power");

        telemetry.update();
    }
}
