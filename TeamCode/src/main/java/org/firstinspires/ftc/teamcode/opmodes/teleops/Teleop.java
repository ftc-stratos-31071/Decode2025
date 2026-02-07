package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.IntakeSeqCmd;
import org.firstinspires.ftc.teamcode.commands.RapidFireCmd;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LaserRangefinder;
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

@TeleOp(name = "Teleop")
public class Teleop extends NextFTCOpMode {

    public Teleop() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    public static double TRACKING_GAIN = 0.08;  // Reduced from 0.15 - much smoother
    public static double SMOOTHING = 0.5;  // Exponential smoothing (0.0-1.0, lower = smoother)
    public static double DEADBAND = 3.0;  // Increased from 2.0 - larger tolerance to prevent jitter
    public static boolean AUTO_TRACK_ENABLED = false;  // Enable/disable tracking
    public static double NO_TARGET_TIMEOUT_SEC = 0.5;  // Time before returning to center when no target detected

    private final MotorEx frontLeftMotor = new MotorEx("frontLeftMotor").brakeMode().reversed();
    private final MotorEx frontRightMotor = new MotorEx("frontRightMotor").brakeMode().reversed();
    private final MotorEx backLeftMotor = new MotorEx("backLeftMotor").brakeMode();
    private final MotorEx backRightMotor = new MotorEx("backRightMotor").brakeMode();
    private LaserRangefinder rangefinder;
    private int ballCount = 0;
    private boolean ballPresent = false;
    private long lastBallTime = 0;
    private static final double BALL_DISTANCE_MM = 40;
    private static final long DEBOUNCE_MS = 300;
    private boolean intakeActive = false;
    private boolean hasRumbled = false;

    private boolean slowMode = false;
    private double driveScale = 1.0;
    private boolean shooterOn = false;
    private double hoodPos = ShooterConstants.servoPos;
    private double turretPos = 180.0;
    private double targetRpm = ShooterConstants.closeTargetRPM;

    Limelight3A limelight;
    double motorTargetX = 180.0;
    double smoothedTx = 0.0;  // Smoothed TX value to prevent jitter
    boolean hasSeenTarget = false;  // Track if we've ever seen a target
    long lastTargetSeenTime = 0;  // Timestamp of last valid target detection
    public static boolean SHOW_LIMELIGHT_TELEMETRY = false;

    @Override
    public void onInit() {
        Shooter.INSTANCE.setHood(hoodPos).schedule();
        Shooter.INSTANCE.setTargetRPM(0.0);
        Shooter.INSTANCE.runRPM(0.0).schedule();
        Turret.INSTANCE.setTurretAngleDeg(180.0);
        Turret.INSTANCE.goToAngle(180.0).schedule();
        Intake.INSTANCE.moveServoPos().schedule();

        FtcDashboard dashboard = FtcDashboard.getInstance();

        motorTargetX = 180.0;
        smoothedTx = 0.0;
        hasSeenTarget = false;
        lastTargetSeenTime = System.currentTimeMillis();

        RevColorSensorV3 sensor = hardwareMap.get(RevColorSensorV3.class, "range");
        rangefinder = new LaserRangefinder(sensor);
        rangefinder.setDistanceMode(LaserRangefinder.DistanceMode.SHORT);
        rangefinder.setTiming(20, 0);

        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.start();
            limelight.pipelineSwitch(0);

            dashboard.startCameraStream(limelight, 0);


        } catch (Exception e) {
        }



    }

    @Override
    public void onStartButtonPressed() {
        Intake.INSTANCE.moveServoPos().schedule();

        var forward = Gamepads.gamepad1().leftStickY().negate().map(v -> v * driveScale);
        var strafe  = Gamepads.gamepad1().leftStickX().map(v -> v * driveScale);
        var rotate  = Gamepads.gamepad1().rightStickX().map(v -> v * driveScale);

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

        Gamepads.gamepad1().leftBumper().whenBecomesTrue(() -> {
            intakeActive = true;
            hasRumbled = false;
            ballCount = 0;
            Intake.INSTANCE.moveServoPos().schedule();
            IntakeSeqCmd.create().schedule();
        });

        Gamepads.gamepad1().leftBumper().whenBecomesFalse(() -> {
            intakeActive = false;
            ballPresent = false;
            Intake.INSTANCE.zeroPowerIntake().schedule();
            Intake.INSTANCE.zeroPowerTransfer().schedule();
        });

        Gamepads.gamepad1().b().whenBecomesTrue(() -> {
            Intake.INSTANCE.moveIntake(-IntakeConstants.intakePowerSlow).schedule();
            Intake.INSTANCE.moveTransfer(-IntakeConstants.intakePowerSlow).schedule();
            Intake.INSTANCE.defaultPos().schedule();
        });

        Gamepads.gamepad1().b().whenBecomesFalse(() -> {
            Intake.INSTANCE.zeroPowerIntake().schedule();
            Intake.INSTANCE.zeroPowerTransfer().schedule();
        });

        Gamepads.gamepad1().a().whenBecomesTrue(() -> {
            ballCount = 0;
            Intake.INSTANCE.defaultPos().schedule();
            RapidFireCmd.create().schedule();
        });

        Gamepads.gamepad1().a().whenBecomesFalse(() -> {
            Intake.INSTANCE.zeroPowerIntake().schedule();
            Intake.INSTANCE.zeroPowerTransfer().schedule();
        });

        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> {
            shooterOn = !shooterOn;

            if (shooterOn) {
                Shooter.INSTANCE.runRPM(ShooterConstants.closeTargetRPM).schedule();
            } else {
                Shooter.INSTANCE.stop();
            }
        });

        Gamepads.gamepad1().dpadUp().whenBecomesTrue(() -> {
            hoodPos = hoodPos - 0.1;
            Shooter.INSTANCE.setHood(hoodPos).schedule();
        });

        Gamepads.gamepad1().dpadDown().whenBecomesTrue(() -> {
            hoodPos = hoodPos + 0.1;
            Shooter.INSTANCE.setHood(hoodPos).schedule();
        });

        Gamepads.gamepad2().dpadRight().whenBecomesTrue(() -> {
            turretPos = turretPos + 10;
            Turret.INSTANCE.goToAngle(turretPos).schedule();
        });

        Gamepads.gamepad2().dpadLeft().whenBecomesTrue(() -> {
            turretPos = turretPos - 10;
            Turret.INSTANCE.goToAngle(turretPos).schedule();
        });

        Gamepads.gamepad1().dpadRight().whenBecomesTrue(() -> {
            targetRpm = targetRpm + 100;
            if (Shooter.INSTANCE.getRPM() > 0) {
                Shooter.INSTANCE.setTargetRPM(targetRpm);
            }
        });

        Gamepads.gamepad1().dpadLeft().whenBecomesTrue(() -> {
            targetRpm = targetRpm - 100;
            if (Shooter.INSTANCE.getRPM() > 0) {
                Shooter.INSTANCE.setTargetRPM(targetRpm);
            }
        });
    }

    @Override
    public void onUpdate() {
        double distance = rangefinder.getDistance(DistanceUnit.MM);
        long now = System.currentTimeMillis();

        boolean detected = distance < BALL_DISTANCE_MM;

        if (intakeActive && detected && !ballPresent && now - lastBallTime > DEBOUNCE_MS) {
            ballCount++;
            lastBallTime = now;
        }

        ballPresent = detected;

        if (ballCount >= 3 && !hasRumbled) {
            Gamepads.gamepad1().getGamepad().invoke().rumble(500);
            hasRumbled = true;
        }

        if (!intakeActive && ballCount >= 3) {
            ballCount = 0;
            hasRumbled = false;
        }

        LLResult result = null;
        if (limelight != null) {
            try {
                result = limelight.getLatestResult();
            } catch (Exception e) {
                if (SHOW_LIMELIGHT_TELEMETRY) {
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
//                        if (Math.abs(smoothedTx) < 10.0) {
//                            adjustment *= 0.5;  // Half speed when within 10 degrees
//                        }


                        motorTargetX += adjustment;
                    }
                    // If within deadband, keep current target (don't update)


                    if (SHOW_LIMELIGHT_TELEMETRY) {








                    }
                }
            } else {
                if (SHOW_LIMELIGHT_TELEMETRY) {

                }
                // Keep last known position if we've seen a target before
                if (!hasSeenTarget) {
                    motorTargetX = 180.0;  // Return to center if never seen target
                    smoothedTx = 0.0;
                }
            }
        } else if (!AUTO_TRACK_ENABLED) {
            if (SHOW_LIMELIGHT_TELEMETRY) {


            }
            smoothedTx = 0.0;  // Reset smoothing when disabled
        } else {
            if (SHOW_LIMELIGHT_TELEMETRY) {


            }
            // Keep last position or return to center if never tracked
            if (!hasSeenTarget) {
                motorTargetX = 180.0;
                smoothedTx = 0.0;
            }
        }


        // Check for timeout - return turret to center if no target detected for a while
        if (System.currentTimeMillis() - lastTargetSeenTime > NO_TARGET_TIMEOUT_SEC * 1000) {
            motorTargetX = 180.0;  // Return to center (front)
            smoothedTx = 0.0;
            hasSeenTarget = false;
        }

        Turret.INSTANCE.goToAngle(motorTargetX).schedule();






        double rightRPM = Shooter.INSTANCE.ticksPerSecondToRPM(Math.abs(Shooter.INSTANCE.rightMotor.getVelocity()));
        double leftRPM = Shooter.INSTANCE.ticksPerSecondToRPM(Math.abs(Shooter.INSTANCE.leftMotor.getVelocity()));
        double currentRPM = (rightRPM + leftRPM) / 2.0;




    }
}
