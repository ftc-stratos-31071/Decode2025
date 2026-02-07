package org.firstinspires.ftc.teamcode.opmodes.teleops;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commands.IntakeSeqCmd;
import org.firstinspires.ftc.teamcode.commands.RapidFireCmd;
import org.firstinspires.ftc.teamcode.constants.AutoPoseMemory;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
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
@TeleOp(name = "RedTeleop")
public class RedTeleop extends NextFTCOpMode {

    public RedTeleop() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Turret2.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    // ═══════════════════════════════════════════════════════════════════
    // TURRET TRACKING SETTINGS (from CompTurretSystem)
    // ═══════════════════════════════════════════════════════════════════

    public static boolean AUTO_TRACK_ENABLED = true;
    public static int TARGET_TAG_ID = 24;

    // Red Goal position (inches) - DECODE field coordinate system
    public static double RED_GOAL_X = -72.0;
    public static double RED_GOAL_Y = 72.0;

    // Starting position
    public static double START_X = 0.0;
    public static double START_Y = 0.0;
    public static double START_HEADING = 90.0;
    public static boolean USE_AUTO_START_POSE = false;
    public static double AUTO_START_X = 0.0;
    public static double AUTO_START_Y = 0.0;
    public static double AUTO_START_HEADING = 90.0;

    // Vision tracking settings
    public static double VISION_TRACKING_GAIN = 0.3;
    public static double VISION_TIMEOUT_SEC = 0.5;
    public static double VISION_DEADBAND_DEG = 15.0;
    public static double VISION_SMOOTHING = 0.5;
    public static double TAG_SEARCH_TRIGGER_THRESHOLD = 0.6;
    public static double TRIGGER_SEARCH_START_ANGLE_DEG = 60.0;

    // ═══════════════════════════════════════════════════════════════════
    // HARDWARE (from BlueTeleop)
    // ═══════════════════════════════════════════════════════════════════

    private final MotorEx frontLeftMotor  = new MotorEx("frontLeftMotor").brakeMode().reversed();
    private final MotorEx frontRightMotor = new MotorEx("frontRightMotor").brakeMode().reversed();
    private final MotorEx backLeftMotor   = new MotorEx("backLeftMotor").brakeMode();
    private final MotorEx backRightMotor  = new MotorEx("backRightMotor").brakeMode();

    private GoBildaPinpointDriver pinpoint;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // ═══════════════════════════════════════════════════════════════════
    // STATE VARIABLES
    // ═══════════════════════════════════════════════════════════════════

    // Drive controls (from BlueTeleop)
    private boolean slowMode = false;
    private double driveScale = 1.0;

    // Shooter controls (from BlueTeleop)
    private boolean shooterOn = false;
    private boolean farOn = false;
    private double hoodPos = ShooterConstants.servoPos;
    private double targetRpm = ShooterConstants.closeTargetRPM;

    // Turret tracking (from CompTurretSystem)
    private boolean trackingEnabled = true;
    private boolean visionOnlyMode = false;
    private boolean visionMode = false;
    private long lastTagSeenTime = 0;
    private double targetGlobalHeading = 0.0;
    private double lastVisionAngle = 0.0;
    private double smoothedTurretAngle = 0.0;
    private boolean poseCalibrated = false;
    private boolean startedFromAutoPose = false;
    private double initPoseFtcX = 0.0;
    private double initPoseFtcY = 0.0;
    private double initPoseHeading = 0.0;
    private String initPoseSource = "UNKNOWN";
    private boolean waitingForLateAutoPose = false;
    private boolean lateAutoPoseApplied = false;
    private long trackingResumeAtMs = 0L;
    private boolean prevLeftTriggerPressed = false;
    private boolean prevRightTriggerPressed = false;
    private int triggerSearchDirection = 0;
    private double triggerSearchBaseAngle = 0.0;

    @Override
    public void onInit() {
        // Initialize odometry (from CompTurretSystem)
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

        // Initialize shooter and turret (from BlueTeleop)
        Shooter.INSTANCE.setHood(hoodPos).schedule();
        Shooter.INSTANCE.setTargetRPM(0.0);
        Shooter.INSTANCE.runRPM(0.0).schedule();
        Turret2.INSTANCE.setAngle(0.0);
        Intake.INSTANCE.moveServoPos().schedule();

        // Set up FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Configure AprilTag processor (from CompTurretSystem)
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(530.423, 530.423, 447.938, 335.553)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        // Build vision portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "arducam"))
                .setCameraResolution(new Size(800, 600))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .build();

        dashboard.startCameraStream(visionPortal, 30);

        lastTagSeenTime = System.currentTimeMillis();

        telemetry.addData("Status", "✓ Initialized - RedTeleop");
        telemetry.addData("Turret Tracking", AUTO_TRACK_ENABLED ? "ENABLED" : "DISABLED");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        Intake.INSTANCE.moveServoPos().schedule();
        Turret2.INSTANCE.setAngle(0.0);

        // Set up mecanum drive control
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

        // Left stick button: Toggle slow mode
        Gamepads.gamepad1().leftStickButton().whenBecomesTrue(() -> {
            slowMode = !slowMode;
            driveScale = slowMode ? 0.25 : 1.0;
        });

        // Left bumper: Intake sequence
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(() -> {
            Intake.INSTANCE.moveServoPos().schedule();
            IntakeSeqCmd.create().schedule();
        });

        Gamepads.gamepad1().leftBumper().whenBecomesFalse(() -> {
            Intake.INSTANCE.zeroPowerIntake().schedule();
            Intake.INSTANCE.zeroPowerTransfer().schedule();
        });

        // B button: Reverse intake (outtake)
        Gamepads.gamepad1().b().whenBecomesTrue(() -> {
            Intake.INSTANCE.moveIntake(-IntakeConstants.intakePowerSlow).schedule();
            Intake.INSTANCE.moveTransfer(-IntakeConstants.intakePowerSlow).schedule();
            Intake.INSTANCE.defaultPos().schedule();
        });

        Gamepads.gamepad1().b().whenBecomesFalse(() -> {
            Intake.INSTANCE.zeroPowerIntake().schedule();
            Intake.INSTANCE.zeroPowerTransfer().schedule();
        });

        // A button: Rapid fire
        Gamepads.gamepad1().a().whenBecomesTrue(() -> {
            Intake.INSTANCE.defaultPos().schedule();
            RapidFireCmd.create().schedule();
        });

        Gamepads.gamepad1().a().whenBecomesFalse(() -> {
            Intake.INSTANCE.zeroPowerIntake().schedule();
            Intake.INSTANCE.zeroPowerTransfer().schedule();
        });

        // Right bumper: Toggle shooter
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> {
            shooterOn = !shooterOn;
            if (shooterOn) {
                Shooter.INSTANCE.runRPM(ShooterConstants.closeTargetRPM).schedule();
            } else {
                Shooter.INSTANCE.stop();
            }
        });

        // DPad Up/Down: Adjust hood
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(() -> {
            hoodPos = hoodPos - 0.1;
            Shooter.INSTANCE.setHood(hoodPos).schedule();
        });

        Gamepads.gamepad1().dpadDown().whenBecomesTrue(() -> {
            hoodPos = hoodPos + 0.1;
            Shooter.INSTANCE.setHood(hoodPos).schedule();
        });

        // Right stick button: Toggle far shot
        Gamepads.gamepad1().rightStickButton().whenBecomesTrue(() -> {
            farOn = !farOn;
            if (farOn) {
                targetRpm = ShooterConstants.farTargetRPM;
                Shooter.INSTANCE.runRPM(ShooterConstants.farTargetRPM).schedule();
                hoodPos = ShooterConstants.FAR_MODE_HOOD_POS;
                Shooter.INSTANCE.setHood(hoodPos).schedule();
            } else {
                targetRpm = ShooterConstants.closeTargetRPM;
                Shooter.INSTANCE.runRPM(ShooterConstants.closeTargetRPM).schedule();
                hoodPos = ShooterConstants.servoPos;
                Shooter.INSTANCE.setHood(hoodPos).schedule();
            }
        });

        // Gamepad2 bumpers: explicit mode select
        Gamepads.gamepad2().leftBumper().whenBecomesTrue(() -> {
            farOn = false;
            targetRpm = ShooterConstants.closeTargetRPM;
            hoodPos = ShooterConstants.servoPos;
            Shooter.INSTANCE.setHood(hoodPos).schedule();
            if (shooterOn) {
                Shooter.INSTANCE.runRPM(targetRpm).schedule();
            }
        });

        Gamepads.gamepad2().rightBumper().whenBecomesTrue(() -> {
            farOn = true;
            targetRpm = ShooterConstants.farTargetRPM;
            hoodPos = ShooterConstants.FAR_MODE_HOOD_POS;
            Shooter.INSTANCE.setHood(hoodPos).schedule();
            if (shooterOn) {
                Shooter.INSTANCE.runRPM(targetRpm).schedule();
            }
        });

        // Y button: center turret, clear trigger-search latch, and wait before tracking resumes.
        Gamepads.gamepad1().y().whenBecomesTrue(() -> {
            trackingEnabled = true;
            visionMode = false;
            lastVisionAngle = 0.0;
            smoothedTurretAngle = 0.0;
            triggerSearchDirection = 0;
            triggerSearchBaseAngle = 0.0;
            trackingResumeAtMs = System.currentTimeMillis() + (long) (VISION_TIMEOUT_SEC * 1000.0);
            Turret2.INSTANCE.setAngle(0.0);
        });

        // DPad Left/Right: Adjust RPM
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

        // Update turret tracking (from CompTurretSystem)
        if (AUTO_TRACK_ENABLED && trackingEnabled) {
            updateTurretTracking();
        }

        // Display telemetry
        displayTelemetry();
    }

    /**
     * Update turret tracking using hybrid odometry + vision system (from CompTurretSystem)
     */
    private void updateTurretTracking() {
        // Update odometry
        pinpoint.update();
        Pose2D currentPose = pinpoint.getPosition();

        // Invert X and Y to match DECODE field coordinate system
        double currentX = -currentPose.getX(DistanceUnit.INCH);
        double currentY = -currentPose.getY(DistanceUnit.INCH);
        double currentRobotHeading = currentPose.getHeading(AngleUnit.DEGREES);

        // Get current goal position
        double goalX = RED_GOAL_X;
        double goalY = RED_GOAL_Y;

        // ALWAYS update targetGlobalHeading using atan2 (odometry runs continuously)
        targetGlobalHeading = calculateAngleToGoal(currentX, currentY, goalX, goalY);

        // Draw field visualization
        drawFieldVisualization(currentX, currentY, currentRobotHeading, goalX, goalY);

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

        double targetTurretAngle;

        if (System.currentTimeMillis() < trackingResumeAtMs) {
            visionMode = false;
            if (triggerSearchDirection != 0) {
                targetTurretAngle = triggerSearchBaseAngle;
                Turret2.INSTANCE.setAngle(targetTurretAngle);
            } else {
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
                targetTurretAngle = calculateTurretAngle(currentRobotHeading, targetGlobalHeading);
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
                double tagFieldX = RED_GOAL_X;
                double tagFieldY = RED_GOAL_Y;

                double rangeToTag = tag.ftcPose.range;
                double bearingToTag = Math.toRadians(tag.ftcPose.bearing);

                double currentHeading = pinpoint.getHeading(AngleUnit.RADIANS);
                double globalBearing = currentHeading + bearingToTag;
                double dx = rangeToTag * Math.cos(globalBearing);
                double dy = rangeToTag * Math.sin(globalBearing);

                double robotX = tagFieldX - dx;
                double robotY = tagFieldY - dy;

                double currentHeadingDeg = pinpoint.getHeading(AngleUnit.DEGREES);
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, robotX, robotY, AngleUnit.DEGREES, currentHeadingDeg));

                poseCalibrated = true;
                Gamepads.gamepad1().getGamepad().invoke().rumble(200);
                break;
            }
        }
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
     * Draw field visualization on FTC Dashboard
     */
    private void drawFieldVisualization(double currentX, double currentY, double currentRobotHeading,
                                        double goalX, double goalY) {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        // Draw goal
        String goalColor = "#FF0000";
        fieldOverlay.setStroke(goalColor);
        fieldOverlay.setStrokeWidth(2);
        double goalSize = 4;
        fieldOverlay.strokeLine(goalX - goalSize, goalY - goalSize, goalX + goalSize, goalY + goalSize);
        fieldOverlay.strokeLine(goalX - goalSize, goalY + goalSize, goalX + goalSize, goalY - goalSize);
        fieldOverlay.strokeCircle(goalX, goalY, 6);

        // Draw robot
        fieldOverlay.setStroke("#0000FF");
        fieldOverlay.setFill("#0000FF");
        fieldOverlay.fillCircle(currentX, currentY, 6);

        // Draw robot heading
        double headingRadians = Math.toRadians(currentRobotHeading);
        double arrowLength = 12;
        double arrowEndX = currentX + arrowLength * Math.cos(headingRadians);
        double arrowEndY = currentY + arrowLength * Math.sin(headingRadians);
        fieldOverlay.strokeLine(currentX, currentY, arrowEndX, arrowEndY);

        // Draw line to goal
        fieldOverlay.setStroke("#00FF00");
        fieldOverlay.setStrokeWidth(1);
        fieldOverlay.strokeLine(currentX, currentY, goalX, goalY);

        // Draw turret direction
        if (trackingEnabled) {
            double turretLogical = Turret2.INSTANCE.getTargetLogicalDeg();
            double turretGlobalHeading = currentRobotHeading - turretLogical;
            double turretRadians = Math.toRadians(turretGlobalHeading);
            double turretLength = 18;
            double turretEndX = currentX + turretLength * Math.cos(turretRadians);
            double turretEndY = currentY + turretLength * Math.sin(turretRadians);

            fieldOverlay.setStroke(visionMode ? "#FFA500" : "#FFFF00");
            fieldOverlay.setStrokeWidth(3);
            fieldOverlay.strokeLine(currentX, currentY, turretEndX, turretEndY);
        }

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
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

    /**
     * Display telemetry
     */
    private void displayTelemetry() {
        telemetry.addLine("═══ RED TELEOP ═══");
        telemetry.addLine();

        // Shooter info
        double rightRPM = Shooter.INSTANCE.ticksPerSecondToRPM(Math.abs(Shooter.INSTANCE.rightMotor.getVelocity()));
        double leftRPM = Shooter.INSTANCE.ticksPerSecondToRPM(Math.abs(Shooter.INSTANCE.leftMotor.getVelocity()));
        double currentRPM = (rightRPM + leftRPM) / 2.0;
        telemetry.addData("Current RPM", currentRPM);
        telemetry.addData("Hood Position", hoodPos);
        telemetry.addLine();

        // Turret tracking
        if (AUTO_TRACK_ENABLED && trackingEnabled) {
            telemetry.addData("Tracking Mode", visionMode ? "🎯 VISION" : "🧭 ODOMETRY");
            telemetry.addData("Goal", "RED");
        } else {
            telemetry.addData("Tracking", "DISABLED");
        }
        telemetry.addData("Vision Only Mode", visionOnlyMode ? "YES" : "NO");
        telemetry.addData("Turret Angle", "%.1f°", Turret2.INSTANCE.getCurrentLogicalDeg());
        telemetry.addData("Pose Calibrated", poseCalibrated ? "✓" : "✗");
        telemetry.addData("AutoPose Used", startedFromAutoPose ? "YES" : "NO");
        telemetry.addData("Init Pose Source", initPoseSource);
        telemetry.addData("Init Pose FTC", "(%.1f, %.1f, %.1f°)", initPoseFtcX, initPoseFtcY, initPoseHeading);
        telemetry.addData("Late AutoPose Applied", lateAutoPoseApplied ? "YES" : "NO");
        telemetry.addData("AutoPose Memory", "has=%s (%.1f, %.1f, %.1f°)",
                AutoPoseMemory.hasPose, AutoPoseMemory.ftcX, AutoPoseMemory.ftcY, AutoPoseMemory.headingDeg);

        telemetry.update();
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
