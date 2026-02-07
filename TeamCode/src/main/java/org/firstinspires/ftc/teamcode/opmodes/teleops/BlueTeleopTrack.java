package org.firstinspires.ftc.teamcode.opmodes.teleops;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.IntakeSeqCmd;
import org.firstinspires.ftc.teamcode.commands.RapidFireCmd;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LaserRangefinder;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret2;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@Config
@TeleOp(name = "BlueTeleopTrack")
public class BlueTeleopTrack extends NextFTCOpMode {

    public BlueTeleopTrack() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Turret2.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    // TURRET TRACKING SETTINGS (from CompTurretSystem)
    public static boolean AUTO_TRACK_ENABLED = true;
    public static int TARGET_TAG_ID = 20;

    // Vision tracking settings
    public static double VISION_TRACKING_GAIN = 0.3; //0.1
    public static double VISION_TIMEOUT_SEC = 1.0; //0.0.5
    public static double VISION_DEADBAND_DEG = 15.0; //10.0
    public static double VISION_SMOOTHING = 0.6; //0.3
    public static double TAG_SEARCH_TRIGGER_THRESHOLD = 0.6;
    public static double TRIGGER_SEARCH_START_ANGLE_DEG = 70.0;

    private final MotorEx frontLeftMotor  = new MotorEx("frontLeftMotor").brakeMode().reversed();
    private final MotorEx frontRightMotor = new MotorEx("frontRightMotor").brakeMode().reversed();
    private final MotorEx backLeftMotor   = new MotorEx("backLeftMotor").brakeMode();
    private final MotorEx backRightMotor  = new MotorEx("backRightMotor").brakeMode();

    private LaserRangefinder rangefinder;
    private int ballCount = 0;
    private boolean ballPresent = false;
    private long lastBallTime = 0;

    private static final double BALL_DISTANCE_MM = 30;
    private static final long DEBOUNCE_MS = 400;

    private boolean slowMode = false;
    private double driveScale = 1.0;

    private boolean shooterOn = false;
    private boolean farOn = false;
    private double hoodPos = ShooterConstants.servoPos;
    private double targetRpm = ShooterConstants.closeTargetRPM;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private boolean ballCountingEnabled = true;
    // Turret tracking (from CompTurretSystem)
    private boolean trackingEnabled = true;
    private boolean visionMode = false;
    private long lastTagSeenTime = 0;
    private double targetTurretAngle = 0.0;
    private double lastVisionAngle = 0.0;
    private double smoothedTurretAngle = 0.0;
    private long trackingResumeAtMs = 0L;
    private boolean prevLeftTriggerPressed = false;
    private boolean prevRightTriggerPressed = false;
    private int triggerSearchDirection = 0;
    private double triggerSearchBaseAngle = 0.0;



    @Override
    public void onInit() {
        Shooter.INSTANCE.setHood(hoodPos).schedule();
        Shooter.INSTANCE.setTargetRPM(0.0);
        Shooter.INSTANCE.runRPM(0.0).schedule();

        Turret2.INSTANCE.setAngle(0.0);
        Intake.INSTANCE.moveServoPos().schedule();

        RevColorSensorV3 sensor = hardwareMap.get(RevColorSensorV3.class, "range");
        rangefinder = new LaserRangefinder(sensor);
        rangefinder.setDistanceMode(LaserRangefinder.DistanceMode.SHORT);
        rangefinder.setTiming(20, 0);

        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(530.423, 530.423, 447.938, 335.553)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "arducam"))
                .setCameraResolution(new Size(800, 600))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .build();

        lastTagSeenTime = System.currentTimeMillis();
    }

    @Override
    public void onStartButtonPressed() {
        Intake.INSTANCE.moveServoPos().schedule();
        Turret2.INSTANCE.setAngle(0.0);

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
            Intake.INSTANCE.moveServoPos().schedule();
            IntakeSeqCmd.create().schedule();
        });

        Gamepads.gamepad1().leftBumper().whenBecomesFalse(() -> {
            Intake.INSTANCE.zeroPowerIntake().schedule();
            Intake.INSTANCE.zeroPowerTransfer().schedule();
        });

        Gamepads.gamepad1().b().whenBecomesTrue(() -> {
            ballCountingEnabled = false;

            Intake.INSTANCE.moveIntake(-IntakeConstants.intakePowerSlow).schedule();
            Intake.INSTANCE.moveTransfer(-IntakeConstants.intakePowerSlow).schedule();
            Intake.INSTANCE.defaultPos().schedule();
        });

        Gamepads.gamepad1().b().whenBecomesFalse(() -> {
            ballCountingEnabled = true;

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

        Gamepads.gamepad1().rightStickButton().whenBecomesTrue(() -> {
            farOn = !farOn;

            if (farOn) {
                targetRpm = ShooterConstants.farTargetRPM;
                Shooter.INSTANCE.runRPM(ShooterConstants.farTargetRPM).schedule();
                hoodPos = ShooterConstants.FAR_MODE_HOOD_POS;
                Shooter.INSTANCE.setHood(hoodPos).schedule();
            }
            else {
                targetRpm = ShooterConstants.closeTargetRPM;
                Shooter.INSTANCE.runRPM(ShooterConstants.closeTargetRPM).schedule();
                hoodPos = ShooterConstants.servoPos;
                Shooter.INSTANCE.setHood(hoodPos).schedule();
            }
        });

        Gamepads.gamepad1().y().whenBecomesTrue(() -> {
            trackingEnabled = true;
            visionMode = false;
            targetTurretAngle = 0.0;
            lastVisionAngle = 0.0;
            smoothedTurretAngle = 0.0;
            triggerSearchDirection = 0;
            triggerSearchBaseAngle = 0.0;
            trackingResumeAtMs = System.currentTimeMillis() + (long) (VISION_TIMEOUT_SEC * 1000.0);
            Turret2.INSTANCE.setAngle(0.0);
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

        // Gamepad2 Y: center turret and reset vision tracking state.
        Gamepads.gamepad2().y().whenBecomesTrue(() -> {
            trackingEnabled = true;
            visionMode = false;
            targetTurretAngle = 0.0;
            lastVisionAngle = 0.0;
            smoothedTurretAngle = 0.0;
            triggerSearchDirection = 0;
            triggerSearchBaseAngle = 0.0;
            trackingResumeAtMs = System.currentTimeMillis() + (long) (VISION_TIMEOUT_SEC * 1000.0);
            Turret2.INSTANCE.setAngle(0.0);
        });
    }

    @Override
    public void onUpdate() {
        double distance = rangefinder.getDistance(DistanceUnit.MM);
        long now = System.currentTimeMillis();
        boolean detected = distance < BALL_DISTANCE_MM;

        if (ballCountingEnabled) {
            if (detected && !ballPresent && now - lastBallTime > DEBOUNCE_MS) {
                ballCount++;

                if (ballCount > 3) {
                    ballCount = 1;
                }

                if (ballCount == 3) {
                    Gamepads.gamepad1().getGamepad().invoke().rumble(500);
                }

                lastBallTime = now;
            }

            ballPresent = detected;
        }

        if (trackingEnabled) {
            updateTurretTracking();
        }

        displayTelemetry(distance);
    }

    /**
     * Vision-only turret tracking with trigger-latched search angles.
     */
    private void updateTurretTracking() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        boolean tagDetectedThisFrame = false;
        double tagBearing = 0.0;

        if (!detections.isEmpty()) {
            for (AprilTagDetection tag : detections) {
                if (tag.id == TARGET_TAG_ID) {
                    tagDetectedThisFrame = true;
                    tagBearing = tag.ftcPose.bearing;
                    lastTagSeenTime = System.currentTimeMillis();
                    break;
                }
            }
        }

        var gamepad1 = Gamepads.gamepad1().getGamepad().invoke();
        boolean leftTriggerPressed = gamepad1.left_trigger > TAG_SEARCH_TRIGGER_THRESHOLD;
        boolean rightTriggerPressed = gamepad1.right_trigger > TAG_SEARCH_TRIGGER_THRESHOLD;

        if (leftTriggerPressed && !prevLeftTriggerPressed) {
            triggerSearchDirection = -1;
            triggerSearchBaseAngle = -Math.abs(TRIGGER_SEARCH_START_ANGLE_DEG);
            smoothedTurretAngle = triggerSearchBaseAngle;
            lastVisionAngle = triggerSearchBaseAngle;
            trackingResumeAtMs = System.currentTimeMillis() + (long) (VISION_TIMEOUT_SEC * 1000.0);
        }
        if (rightTriggerPressed && !prevRightTriggerPressed) {
            triggerSearchDirection = 1;
            triggerSearchBaseAngle = Math.abs(TRIGGER_SEARCH_START_ANGLE_DEG);
            smoothedTurretAngle = triggerSearchBaseAngle;
            lastVisionAngle = triggerSearchBaseAngle;
            trackingResumeAtMs = System.currentTimeMillis() + (long) (VISION_TIMEOUT_SEC * 1000.0);
        }
        prevLeftTriggerPressed = leftTriggerPressed;
        prevRightTriggerPressed = rightTriggerPressed;

        if (System.currentTimeMillis() < trackingResumeAtMs) {
            visionMode = false;
            if (triggerSearchDirection != 0) {
                targetTurretAngle = triggerSearchBaseAngle;
                Turret2.INSTANCE.setAngle(targetTurretAngle);
            } else {
                targetTurretAngle = 0.0;
                Turret2.INSTANCE.setAngle(0.0);
            }
            return;
        }

        if (triggerSearchDirection != 0) {
            if (tagDetectedThisFrame) {
                visionMode = true;

                if (Math.abs(tagBearing) > VISION_DEADBAND_DEG) {
                    double currentTurretAngle = Turret2.INSTANCE.getTargetLogicalDeg();
                    double correction = -tagBearing * VISION_TRACKING_GAIN;
                    double desiredAngle = currentTurretAngle + correction;

                    if (smoothedTurretAngle == 0.0) {
                        smoothedTurretAngle = currentTurretAngle;
                    }

                    targetTurretAngle = smoothedTurretAngle * VISION_SMOOTHING + desiredAngle * (1.0 - VISION_SMOOTHING);
                    targetTurretAngle = Math.max(-Turret2.MAX_ROTATION, Math.min(Turret2.MAX_ROTATION, targetTurretAngle));
                    lastVisionAngle = targetTurretAngle;
                    smoothedTurretAngle = targetTurretAngle;
                } else {
                    targetTurretAngle = Turret2.INSTANCE.getTargetLogicalDeg();
                    lastVisionAngle = targetTurretAngle;
                    smoothedTurretAngle = targetTurretAngle;
                }
            } else {
                visionMode = false;
                targetTurretAngle = triggerSearchBaseAngle;
            }

            targetTurretAngle = Math.max(-Turret2.MAX_ROTATION, Math.min(Turret2.MAX_ROTATION, targetTurretAngle));
            Turret2.INSTANCE.setAngle(targetTurretAngle);
            return;
        }

        if (tagDetectedThisFrame) {
            visionMode = true;

            if (Math.abs(tagBearing) > VISION_DEADBAND_DEG) {
                double currentTurretAngle = Turret2.INSTANCE.getTargetLogicalDeg();
                double correction = -tagBearing * VISION_TRACKING_GAIN;
                double desiredAngle = currentTurretAngle + correction;

                if (smoothedTurretAngle == 0.0) {
                    smoothedTurretAngle = currentTurretAngle;
                }

                targetTurretAngle = smoothedTurretAngle * VISION_SMOOTHING + desiredAngle * (1.0 - VISION_SMOOTHING);
                targetTurretAngle = Math.max(-Turret2.MAX_ROTATION, Math.min(Turret2.MAX_ROTATION, targetTurretAngle));

                lastVisionAngle = targetTurretAngle;
                smoothedTurretAngle = targetTurretAngle;
            } else {
                targetTurretAngle = Turret2.INSTANCE.getTargetLogicalDeg();
                lastVisionAngle = targetTurretAngle;
                smoothedTurretAngle = targetTurretAngle;
            }
        } else if ((System.currentTimeMillis() - lastTagSeenTime) / 1000.0 < VISION_TIMEOUT_SEC) {
            visionMode = true;
            targetTurretAngle = lastVisionAngle;
        } else {
            visionMode = false;
            targetTurretAngle = lastVisionAngle;
        }

        targetTurretAngle = Math.max(-Turret2.MAX_ROTATION, Math.min(Turret2.MAX_ROTATION, targetTurretAngle));
        Turret2.INSTANCE.setAngle(targetTurretAngle);
    }

    private void displayTelemetry(double distanceMm) {




        double rightRPM = Shooter.INSTANCE.ticksPerSecondToRPM(Math.abs(Shooter.INSTANCE.rightMotor.getVelocity()));
        double leftRPM = Shooter.INSTANCE.ticksPerSecondToRPM(Math.abs(Shooter.INSTANCE.leftMotor.getVelocity()));
        double currentRPM = (rightRPM + leftRPM) / 2.0;

















    }

    @Override
    public void onStop() {
        Turret2.INSTANCE.setAngle(0.0);
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
