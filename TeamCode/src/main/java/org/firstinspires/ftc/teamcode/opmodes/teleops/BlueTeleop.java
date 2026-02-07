package org.firstinspires.ftc.teamcode.opmodes.teleops;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.AutoPoseMemory;
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
@TeleOp(name = "BlueTeleop")
public class BlueTeleop extends NextFTCOpMode {

    public BlueTeleop() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Turret2.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    // TURRET TRACKING SETTINGS (from CompTurretSystem)
    public static boolean AUTO_TRACK_ENABLED = true;
    public static int TARGET_TAG_ID = 20;

    // Blue Goal position (inches) - DECODE field coordinate system
    public static double BLUE_GOAL_X = -72.0;
    public static double BLUE_GOAL_Y = -72.0;

    // Starting position
    public static double START_X = 0.0;
    public static double START_Y = 0.0;
    public static double START_HEADING = 90.0;
    public static boolean USE_AUTO_START_POSE = false;
    public static double AUTO_START_X = 0.0;
    public static double AUTO_START_Y = 0.0;
    public static double AUTO_START_HEADING = 90.0;

    // Vision tracking settings
    public static double VISION_TRACKING_GAIN = 0.3; //0.1
    public static double VISION_TIMEOUT_SEC = 1.0; //0.0.5
    public static double VISION_DEADBAND_DEG = 15.0; //10.0
    public static double VISION_SMOOTHING = 0.6; //0.3
    public static double TAG_SEARCH_TRIGGER_THRESHOLD = 0.6;
    public static double TRIGGER_SEARCH_START_ANGLE_DEG = 60.0;

    private final MotorEx frontLeftMotor  = new MotorEx("frontLeftMotor").brakeMode().reversed();
    private final MotorEx frontRightMotor = new MotorEx("frontRightMotor").brakeMode().reversed();
    private final MotorEx backLeftMotor   = new MotorEx("backLeftMotor").brakeMode();
    private final MotorEx backRightMotor  = new MotorEx("backRightMotor").brakeMode();

    private GoBildaPinpointDriver pinpoint;

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
    private boolean visionOnlyMode = false;
    private boolean visionMode = false;
    private long lastTagSeenTime = 0;
    private double targetGlobalHeading = 0.0;
    private double targetTurretAngle = 0.0;
    private double lastVisionAngle = 0.0;
    private double smoothedTurretAngle = 0.0;
    private boolean poseCalibrated = false;
    private boolean startedFromAutoPose = false;
    private double lastRawX = 0.0;
    private double lastRawY = 0.0;
    private double lastFtcX = 0.0;
    private double lastFtcY = 0.0;
    private double lastRawHeading = 0.0;
    private double lastTraditionalHeading = 0.0;
    private String initPoseSource = "UNKNOWN";
    private double initPoseFtcX = 0.0;
    private double initPoseFtcY = 0.0;
    private double initPoseHeading = 0.0;
    private boolean waitingForLateAutoPose = false;
    private boolean lateAutoPoseApplied = false;
    private long trackingResumeAtMs = 0L;
    private boolean prevLeftTriggerPressed = false;
    private boolean prevRightTriggerPressed = false;
    private int triggerSearchDirection = 0;
    private double triggerSearchBaseAngle = 0.0;



    @Override
    public void onInit() {

        // Initialize odometry
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        boolean useSharedAutoPose = AutoPoseMemory.hasPose;
        startedFromAutoPose = useSharedAutoPose;
        initPoseFtcX = useSharedAutoPose ? AutoPoseMemory.ftcX : (USE_AUTO_START_POSE ? AUTO_START_X : START_X);
        initPoseFtcY = useSharedAutoPose ? AutoPoseMemory.ftcY : (USE_AUTO_START_POSE ? AUTO_START_Y : START_Y);
        initPoseHeading = useSharedAutoPose ? AutoPoseMemory.headingDeg : (USE_AUTO_START_POSE ? AUTO_START_HEADING : START_HEADING);
        initPoseSource = useSharedAutoPose ? "AutoPoseMemory@Init" : (USE_AUTO_START_POSE ? "AutoStartConfig" : "TeleopStartConfig");
        waitingForLateAutoPose = !useSharedAutoPose;
        lateAutoPoseApplied = false;
        setPinpointFromTraditional(initPoseFtcX, initPoseFtcY, initPoseHeading);

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

        // X button: Calibrate pose using AprilTag
        Gamepads.gamepad1().x().whenBecomesTrue(this::calibratePoseFromAprilTag);

        // Gamepad2 Y: center turret and switch to vision-only tracking (no odometry fallback).
        Gamepads.gamepad2().y().whenBecomesTrue(() -> {
            trackingEnabled = true;
            visionOnlyMode = true;
            visionMode = false;
            targetTurretAngle = 0.0;
            lastVisionAngle = 0.0;
            smoothedTurretAngle = 0.0;
            triggerSearchDirection = 0;
            triggerSearchBaseAngle = 0.0;
            Turret2.INSTANCE.setAngle(0.0);
        });
    }

    @Override
    public void onUpdate() {
        tryLateAutoPoseHandoff();

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

        updateOdometry();

        if (trackingEnabled) {
            updateTurretTracking();
        }

        displayTelemetry(distance);
    }

    /**
     * Update turret tracking using hybrid odometry + vision system
     */
    private void updateTurretTracking() {
        // Convert FTC-centered pose to Decode coordinates.
        double currentDecodeX = -lastFtcX;
        double currentDecodeY = -lastFtcY;
        targetGlobalHeading = calculateAngleToGoal(currentDecodeX, currentDecodeY, BLUE_GOAL_X, BLUE_GOAL_Y);

        // Check for AprilTag detections
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

        // Calculate time since last tag detection
        double timeSinceLastTag = (System.currentTimeMillis() - lastTagSeenTime) / 1000.0;
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

        // PRIORITY SYSTEM: Vision when tag visible, odometry otherwise
        if (tagDetectedThisFrame) {
            // VISION MODE: Tag is visible
            visionMode = true;

            if (Math.abs(tagBearing) > VISION_DEADBAND_DEG) {
                // Apply vision correction
                double currentTurretAngle = Turret2.INSTANCE.getTargetLogicalDeg();
                double correction = -tagBearing * VISION_TRACKING_GAIN;
                double desiredAngle = currentTurretAngle + correction;

                // Initialize smoothing on first frame
                if (smoothedTurretAngle == 0.0) {
                    smoothedTurretAngle = currentTurretAngle;
                }

                // Apply exponential smoothing
                targetTurretAngle = smoothedTurretAngle * VISION_SMOOTHING + desiredAngle * (1.0 - VISION_SMOOTHING);
                targetTurretAngle = Math.max(-Turret2.MAX_ROTATION, Math.min(Turret2.MAX_ROTATION, targetTurretAngle));

                lastVisionAngle = targetTurretAngle;
                smoothedTurretAngle = targetTurretAngle;
            } else {
                // Within deadband - hold position
                targetTurretAngle = Turret2.INSTANCE.getTargetLogicalDeg();
                lastVisionAngle = targetTurretAngle;
                smoothedTurretAngle = targetTurretAngle;
            }
        } else if (timeSinceLastTag < VISION_TIMEOUT_SEC) {
            // HOLD MODE: Tag recently visible, hold last angle
            visionMode = true;
            targetTurretAngle = lastVisionAngle;
        } else {
            // Tag lost: vision-only and far mode do not use odometry fallback.
            visionMode = false;
            if (visionOnlyMode || farOn) {
                targetTurretAngle = lastVisionAngle;
            } else {
                targetTurretAngle = calculateTurretAngle(lastTraditionalHeading, targetGlobalHeading);
            }
        }
        targetTurretAngle = Math.max(-Turret2.MAX_ROTATION, Math.min(Turret2.MAX_ROTATION, targetTurretAngle));

        // Command turret to target angle
        Turret2.INSTANCE.setAngle(targetTurretAngle);
    }

    /**
     * Calibrate pinpoint pose using AprilTag detection
     */
    private void calibratePoseFromAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        for (AprilTagDetection tag : detections) {
            if (tag.id == TARGET_TAG_ID && tag.metadata != null) {
                double tagFieldX = BLUE_GOAL_X;
                double tagFieldY = BLUE_GOAL_Y;

                double rangeToTag = tag.ftcPose.range;
                double bearingToTag = Math.toRadians(tag.ftcPose.bearing);

                double currentHeading = pinpoint.getHeading(AngleUnit.RADIANS);
                double globalBearing = currentHeading + bearingToTag;
                double dx = rangeToTag * Math.cos(globalBearing);
                double dy = rangeToTag * Math.sin(globalBearing);

                double robotX = tagFieldX - dx;
                double robotY = tagFieldY - dy;

                double robotFtcX = -robotX;
                double robotFtcY = -robotY;

                double currentTraditionalHeadingDeg = pinpoint.getHeading(AngleUnit.DEGREES);
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, robotFtcX, robotFtcY, AngleUnit.DEGREES, currentTraditionalHeadingDeg));

                poseCalibrated = true;
                Gamepads.gamepad1().getGamepad().invoke().rumble(200);
                break;
            }
        }
    }

    private void updateOdometry() {
        pinpoint.update();
        Pose2D currentPose = pinpoint.getPosition();
        lastRawX = currentPose.getX(DistanceUnit.INCH);
        lastRawY = currentPose.getY(DistanceUnit.INCH);
        lastFtcX = lastRawX;
        lastFtcY = lastRawY;
        lastRawHeading = currentPose.getHeading(AngleUnit.DEGREES);
        lastTraditionalHeading = lastRawHeading;
    }

    private void setPinpointFromTraditional(double ftcX, double ftcY, double traditionalHeadingDeg) {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, ftcX, ftcY, AngleUnit.DEGREES, traditionalHeadingDeg));
    }

    private void tryLateAutoPoseHandoff() {
        if (!waitingForLateAutoPose) {
            return;
        }

        if (!AutoPoseMemory.hasPose) {
            return;
        }

        long ageMs = System.currentTimeMillis() - AutoPoseMemory.updatedAtMs;
        if (ageMs > 10_000L) {
            return;
        }

        initPoseFtcX = AutoPoseMemory.ftcX;
        initPoseFtcY = AutoPoseMemory.ftcY;
        initPoseHeading = AutoPoseMemory.headingDeg;
        initPoseSource = "AutoPoseMemory@Late";
        startedFromAutoPose = true;
        waitingForLateAutoPose = false;
        lateAutoPoseApplied = true;
        setPinpointFromTraditional(initPoseFtcX, initPoseFtcY, initPoseHeading);
    }

    /**
     * Calculate angle to goal using atan2
     */
    private double calculateAngleToGoal(double currentX, double currentY, double goalX, double goalY) {
        double deltaX = goalX - currentX;
        double deltaY = goalY - currentY;
        double angleRad = Math.atan2(deltaY, deltaX);
        double angleDeg = Math.toDegrees(angleRad);
        return normalizeAngle(angleDeg);
    }

    /**
     * Calculate turret angle needed to point at target global heading
     */
    private double calculateTurretAngle(double robotHeading, double globalTarget) {
        double angleDiff = globalTarget - robotHeading;
        angleDiff = normalizeAngleSigned(angleDiff);
        double logicalTurretAngle = -angleDiff;
        return Math.max(-Turret2.MAX_ROTATION, Math.min(Turret2.MAX_ROTATION, logicalTurretAngle));
    }

    private void displayTelemetry(double distanceMm) {




        double rightRPM = Shooter.INSTANCE.ticksPerSecondToRPM(Math.abs(Shooter.INSTANCE.rightMotor.getVelocity()));
        double leftRPM = Shooter.INSTANCE.ticksPerSecondToRPM(Math.abs(Shooter.INSTANCE.leftMotor.getVelocity()));
        double currentRPM = (rightRPM + leftRPM) / 2.0;

















    }

    private double normalizeAngle(double degrees) {
        degrees = degrees % 360.0;
        if (degrees < 0) degrees += 360.0;
        return degrees;
    }

    private double normalizeAngleSigned(double degrees) {
        degrees = degrees % 360.0;
        if (degrees > 180.0) degrees -= 360.0;
        else if (degrees < -180.0) degrees += 360.0;
        return degrees;
    }

    @Override
    public void onStop() {
        Turret2.INSTANCE.setAngle(0.0);
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
