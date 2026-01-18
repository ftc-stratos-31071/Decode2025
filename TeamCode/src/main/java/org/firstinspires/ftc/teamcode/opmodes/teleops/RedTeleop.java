package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.IntakeSeqCmd;
import org.firstinspires.ftc.teamcode.commands.KickCmd;
import org.firstinspires.ftc.teamcode.commands.ShootBallSteadyCmd;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@Config
@Disabled
@TeleOp(name = "RedTeleop")
public class RedTeleop extends NextFTCOpMode {
    // Tunable via FTC Dashboard
    public static double TRACKING_GAIN = 0.08;  // Reduced from 0.15 - much smoother
    public static double SMOOTHING = 0.7;  // Exponential smoothing (0.0-1.0, lower = smoother)
    public static double TURRET_LIMIT_DEG = 45.0;  // Max turret rotation
    public static double DEADBAND = 3.0;  // Increased from 2.0 - larger tolerance to prevent jitter
    public static boolean AUTO_TRACK_ENABLED = true;  // Enable/disable tracking
    public static double NO_TARGET_TIMEOUT_SEC = 0.5;  // Time before returning to center when no target detected

    // Telemetry toggles (FTC Dashboard)
    public static boolean SHOW_LIMELIGHT_TELEMETRY = false;
    public static boolean SHOW_SHOOTER_TELEMETRY = false;
    public static boolean SHOW_CONTROLS_TELEMETRY = false;

    // PIDF-based shooter control - adjustable target RPM
    public static double TARGET_RPM = ShooterConstants.closeTargetRPM;  // Target RPM for PIDF control
    public static double RPM_INCREMENT = 100.0;  // How much to adjust RPM per button press
    public static double MIN_TARGET_RPM = 1000.0;
    public static double MAX_TARGET_RPM = 6000.0;

    // Estimated RPM per unit power at full power (adjust if your shooter differs)
    private static final double TARGET_RPM_PER_POWER = 6000.0;

    public RedTeleop() {
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
    double targetRpm = TARGET_RPM;  // Use RPM instead of power


    // Shooter timing
    boolean hasRumbled = false;
    private long shooterStartTime = 0;
    private boolean shooterTiming = false;
    private boolean slowMode = false;
    private double driveScale = 1.0;



    @Override
    public void onInit() {
        Intake.INSTANCE.defaultPos().schedule();
        Shooter.INSTANCE.moveServo(servoPos).schedule();
        Shooter.INSTANCE.kickDefaultPos.schedule();
        Turret.INSTANCE.turret.zeroed();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        Shooter.INSTANCE.stopShooter();

        motorTargetX = 0.0;
        smoothedTx = 0.0;
        hasSeenTarget = false;
        lastTargetSeenTime = System.currentTimeMillis();
        hasRumbled = false;
        shooterTiming = false;
        servoPos = ShooterConstants.defaultPos;
        targetRpm = TARGET_RPM;

        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.start();
            limelight.pipelineSwitch(1);

            dashboard.startCameraStream(limelight, 0);

            telemetry.addData("Limelight", "✓ Connected");
            telemetry.addData("Camera", "✓ Streaming to dashboard");
        } catch (Exception e) {
            telemetry.addData("Limelight", "✗ ERROR: " + e.getMessage());
        }

        telemetry.addData("Status", "Initialized - Press START to begin");
        telemetry.addData("Mode", "Driver Control + Auto Turret");
        telemetry.addData("Turret", "Will center on START");
        telemetry.update();
    }


    @Override
    public void onStartButtonPressed() {
//        Shooter.INSTANCE.setTargetRPM(TARGET_RPM); [need to start Shooter by default]

        var forward = Gamepads.gamepad1().leftStickY().map(v -> v * driveScale);
        var strafe  = Gamepads.gamepad1().leftStickX().negate().map(v -> v * driveScale);
        var rotate  = Gamepads.gamepad1().rightStickX().negate().map(v -> v * driveScale);

        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                forward,
                strafe,
                rotate
        );
        driverControlled.schedule();

        Gamepads.gamepad1().leftStickButton().whenBecomesTrue(() -> {
            slowMode = !slowMode;
            driveScale = slowMode ? 0.25 : 1.0;
        });

        Gamepads.gamepad1().rightStickButton().whenBecomesTrue(() -> {
            targetRpm = (targetRpm == ShooterConstants.closeTargetRPM)
                    ? ShooterConstants.farTargetRPM
                    : ShooterConstants.closeTargetRPM;
            Shooter.INSTANCE.setTargetRPM(targetRpm);
        });

        // Left Bumper - Intake sequence (press to start, release to stop)
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(() -> {
            IntakeSeqCmd.create().schedule();
        });
        Gamepads.gamepad1().leftBumper().whenBecomesFalse(() -> {
            Intake.INSTANCE.zeroPowerIntake().schedule();
        });

        final AtomicBoolean shooterToggle = new AtomicBoolean(false);

        // Right Bumper - Toggle Shooter ON/OFF (press again to turn off)
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> {
            boolean nowOn;
            while (true) {
                boolean prev = shooterToggle.get();
                if (shooterToggle.compareAndSet(prev, !prev)) {
                    nowOn = !prev;
                    break;
                }
            }
            if (nowOn) {
                shooterStartTime = System.currentTimeMillis();
                shooterTiming = true;
                hasRumbled = false;
                Shooter.INSTANCE.setTargetRPM(targetRpm);
            } else {
                Shooter.INSTANCE.setTargetRPM(0);
                shooterTiming = false;
                hasRumbled = false;
            }
        });

        Gamepads.gamepad2().a().whenBecomesTrue(() -> {
            KickCmd.create().schedule();
        });

        Gamepads.gamepad1().a().whenBecomesTrue(() -> {
            Intake.INSTANCE.defaultPos().schedule();
            ShootBallSteadyCmd.create(IntakeConstants.shootPower, ShooterConstants.tolRpm).schedule();
        });

        Gamepads.gamepad1().a().whenBecomesFalse(() -> {
            Intake.INSTANCE.zeroPowerIntake().schedule();
        });

        // B Button - Intake OUTTAKE (reverse direction)
        Gamepads.gamepad1().b().whenBecomesTrue(() -> {
            Intake.INSTANCE.moveIntake(-IntakeConstants.intakePowerSlow).schedule();
            Intake.INSTANCE.defaultPos().schedule();
            Shooter.INSTANCE.kickDefaultPos.schedule();
        });
        Gamepads.gamepad1().b().whenBecomesFalse(() -> {
            Intake.INSTANCE.zeroPowerIntake().schedule();
        });

        final AtomicBoolean xRpmToggle = new AtomicBoolean(false);
        Gamepads.gamepad1().x().whenBecomesTrue(() -> {
            boolean nowOn;
            while (true) {
                boolean prev = xRpmToggle.get();
                if (xRpmToggle.compareAndSet(prev, !prev)) {
                    nowOn = !prev;
                    break;
                }
            }
            targetRpm = nowOn ? 4000 : 3500;
            hasRumbled = false;
            if (Shooter.INSTANCE.getTargetRPM() > 0) {
                Shooter.INSTANCE.setTargetRPM(targetRpm);
            }
        });

        // Shooter RPM adjustment
        Gamepads.gamepad2().dpadRight().whenBecomesTrue(() -> {
            targetRpm = Math.min(MAX_TARGET_RPM, targetRpm + RPM_INCREMENT);
            hasRumbled = false;
            if (Shooter.INSTANCE.getTargetRPM() > 0) {
                Shooter.INSTANCE.setTargetRPM(targetRpm);
            }
        });

        Gamepads.gamepad2().dpadLeft().whenBecomesTrue(() -> {
            targetRpm = Math.max(MIN_TARGET_RPM, targetRpm - RPM_INCREMENT);
            hasRumbled = false;
            if (Shooter.INSTANCE.getTargetRPM() > 0) {
                Shooter.INSTANCE.setTargetRPM(targetRpm);
            }
        });


        // Servo position adjustment with MAX_HOOD_HEIGHT limit
        Gamepads.gamepad2().dpadUp().whenBecomesTrue(() -> {
            servoPos = servoPos - 0.1;
            Shooter.INSTANCE.moveServo(servoPos).schedule();
        });

        Gamepads.gamepad2().dpadDown().whenBecomesTrue(() -> {
            servoPos = servoPos + 0.1;
            Shooter.INSTANCE.moveServo(servoPos).schedule();
        });


        // Y Button - Manual turret reset to center (straight/front)
        Gamepads.gamepad1().y().whenBecomesTrue(() -> {
            motorTargetX = 0.0;
            smoothedTx = 0.0;
            hasSeenTarget = false;
            Turret.INSTANCE.setTargetDegrees(0.0);
            telemetry.addData("Turret", "Reset to Center");
            telemetry.update();
        });
    }


    @Override
    public void onUpdate() {
        // Get current RPM directly from shooter (now matches TuneShooter calculation)
        double currentRpm = Shooter.INSTANCE.getRPM();

        // ========================================================================
        // MODE STATUS (always on)
        // ========================================================================
        telemetry.addData("Drive Mode", slowMode ? "SLOW (0.25x)" : "REGULAR (1.0x)");
        telemetry.addData(
                "Shooter Mode",
                (targetRpm == ShooterConstants.farTargetRPM) ? "FAR" :
                        (targetRpm == ShooterConstants.closeTargetRPM) ? "CLOSE" :
                                "CUSTOM"
        );
        telemetry.addData("Shooter Target RPM", String.format("%.0f", targetRpm));
        telemetry.addData("Shooter Target RPM", String.format("%.0f", currentRpm));


        // RUMBLE WHEN TARGET RPM REACHED (within 5% tolerance)
        if (shooterTiming && !hasRumbled) {
            if (currentRpm >= targetRpm * 0.95) {  // 95% of target = ready to shoot
                try {
                    Gamepads.gamepad1().getGamepad().invoke().rumble(500);
                } catch (Exception ignored) { }
                hasRumbled = true;
            }
        }


        // ========================================================================
        // AUTOMATIC TURRET TRACKING - Tracks CLOSEST AprilTag
        // ========================================================================
        LLResult result = null;
        if (limelight != null) {
            try {
                result = limelight.getLatestResult();
            } catch (Exception e) {
                if (SHOW_LIMELIGHT_TELEMETRY) {
                    telemetry.addData("Limelight Error", e.getMessage());
                }
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


                    if (SHOW_LIMELIGHT_TELEMETRY) {
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
                }
            } else {
                if (SHOW_LIMELIGHT_TELEMETRY) {
                    telemetry.addData("═══ TURRET TRACKING ═══", "");
                    telemetry.addData("Status", "✗ No AprilTags detected");
                }
                // Keep last known position if we've seen a target before
                if (!hasSeenTarget) {
                    motorTargetX = 0.0;  // Return to center if never seen target
                    smoothedTx = 0.0;
                }
            }
        } else if (!AUTO_TRACK_ENABLED) {
            if (SHOW_LIMELIGHT_TELEMETRY) {
                telemetry.addData("═══ TURRET TRACKING ═══", "");
                telemetry.addData("Status", "⚠ DISABLED (Press X to enable)");
                telemetry.addData("Turret Angle", String.format("%.2f°", motorTargetX));
            }
            smoothedTx = 0.0;  // Reset smoothing when disabled
        } else {
            if (SHOW_LIMELIGHT_TELEMETRY) {
                telemetry.addData("═══ TURRET TRACKING ═══", "");
                telemetry.addData("Status", "✗ No valid target");
                telemetry.addData("Turret Angle", String.format("%.2f°", motorTargetX));
            }
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
        if (SHOW_SHOOTER_TELEMETRY) {
            telemetry.addData("", "");
            telemetry.addData("═══ SHOOTER (PIDF) ═══", "");
            telemetry.addData("Error", String.format("%.0f", targetRpm - currentRpm));
            telemetry.addData("Hood Position", String.format("%.2f", servoPos));
            telemetry.addData("PIDF Status", Shooter.INSTANCE.getTargetRPM() > 0 ? "✓ ACTIVE" : "✗ OFF");
            telemetry.addData("PIDF Enabled", Shooter.INSTANCE.isPidfEnabled() ? "YES" : "NO");
            telemetry.addData("Subsystem Target", String.format("%.0f", Shooter.INSTANCE.getTargetRPM()));
        }


        // ========================================================================
        // CONTROLS REMINDER
        // ========================================================================
        if (SHOW_CONTROLS_TELEMETRY) {
            telemetry.addData("", "");
            telemetry.addData("═══ CONTROLS ═══", "");
            telemetry.addData("Left Stick", "Drive");
            telemetry.addData("Right Stick", "Rotate");
            telemetry.addData("X Button", "Shooter Off");
            telemetry.addData("Y Button", "Reset Turret to Center");
            telemetry.addData("Left Bumper", "Intake Sequence");
            telemetry.addData("Right Bumper", "Shooter On");
            telemetry.addData("A Button", "Shoot Ball");
            telemetry.addData("B Button", "Outtake (Reverse Intake)");
            telemetry.addData("DPad Up/Down", "Adjust Hood Position");
            telemetry.addData("DPad Left/Right", "Adjust Target RPM");
        }


        telemetry.update();
    }
}