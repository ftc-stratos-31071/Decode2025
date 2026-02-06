package org.firstinspires.ftc.teamcode.opmodes.teleops;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Turret2;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * CompTurretSystem - Hybrid Odometry + AprilTag Vision Tracking using Turret2
 *
 * TRACKING SYSTEM:
 * This uses a HYBRID approach where BOTH systems work together:
 *
 * 1. ODOMETRY (Base Layer - ALWAYS ACTIVE):
 *    - Uses Pinpoint odometry to continuously track goal position
 *    - Calculates bearing to goal using atan2(deltaY, deltaX)
 *    - Provides general direction to the target
 *
 * 2. VISION (Refinement Layer - WHEN TAG VISIBLE):
 *    - ArduCam scans for AprilTag on the goal
 *    - When tag detected, applies fine-tuning corrections
 *    - Adds small adjustments on top of odometry base angle
 *
 * HOW IT WORKS:
 * - Odometry drives turret to general location of goal
 * - ArduCam provides precise centering when tag is in view
 * - Result: Fast acquisition + Precise tracking
 *
 * DECODE FIELD COORDINATE SYSTEM:
 * - Y axis: Points from Red Wall toward Blue Alliance (across field)
 * - X axis: Points toward audience (front of field)
 * - Red Wall is on the LEFT as seen from audience
 * - 0° heading = facing +X (toward audience)
 * - 90° heading = facing +Y (toward Blue Alliance)
 *
 * FEATURES:
 * - Uses Turret2 subsystem for turret control
 * - Odometry-based goal tracking with atan2
 * - AprilTag-based fine-tuning when tag visible
 * - Button to calibrate pinpoint pose using AprilTag
 * - FTC Dashboard visualization
 *
 * Controls:
 * - Left Stick: Drive forward/backward and strafe
 * - Right Stick X: Rotate robot (turret compensates)
 * - A Button: Toggle tracking on/off
 * - Y Button: Reset turret to center (0°)
 * - X Button: Calibrate pose using AprilTag (if visible)
 * - DPad Up/Down: Switch between Red/Blue goal
 */
@Config
@TeleOp(name = "CompTurretSystem")
public class CompTurretSystem extends NextFTCOpMode {

    private GoBildaPinpointDriver pinpoint;

    // Motor declarations for mecanum drive
    private final MotorEx frontLeftMotor = new MotorEx("frontLeftMotor").brakeMode().reversed();
    private final MotorEx frontRightMotor = new MotorEx("frontRightMotor").brakeMode().reversed();
    private final MotorEx backLeftMotor = new MotorEx("backLeftMotor").brakeMode();
    private final MotorEx backRightMotor = new MotorEx("backRightMotor").brakeMode();

    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // TUNABLE: Target AprilTag ID to track
    public static int TARGET_TAG_ID = 20;

    // TUNABLE: Goal positions (inches)
    //
    // DECODE FIELD COORDINATE SYSTEM:
    // - Y axis: Points from Red Wall toward Blue Alliance
    // - X axis: Points toward audience
    // - Field is 144" x 144" (±72" from center)
    //
    // GOAL POSITIONS:
    // - Blue Goal: On the Blue Alliance side (+Y direction), centered on X
    // - Red Goal: On the Red Wall side (-Y direction), centered on X
    public static double RED_GOAL_X = -72.0;     // Centered on X axis
    public static double RED_GOAL_Y = 72.0;   // Red Wall side (-Y)
    public static double BLUE_GOAL_X = -72.0;    // Centered on X axis
    public static double BLUE_GOAL_Y = -72.0;   // Blue Alliance side (+Y)

    // TUNABLE: Starting robot position (inches)
    // Robot starts in center of field
    public static double START_X = 0.0;        // Center X
    public static double START_Y = 0.0;        // Center Y
    // Robot facing toward Blue goal (+Y direction) = 90°
    // Robot facing toward Red goal (-Y direction) = -90°
    // Robot facing audience (+X direction) = 0°
    public static double START_HEADING = 90.0; // Facing toward Blue goal (adjust based on actual start)

    // TUNABLE: AprilTag tracking settings
    public static double VISION_TRACKING_GAIN = 0.1; // How aggressively to correct toward target (0.0-1.0)
    public static double VISION_TIMEOUT_SEC = 0.5; // Time without tag before odometry takes over
    public static double VISION_DEADBAND_DEG = 10.0; // Don't adjust if within this range (increased for stability)
    public static double VISION_SMOOTHING = 0.3; // Smoothing factor (0.0-1.0, higher = smoother but slower)

    // TUNABLE: Enable/disable tracking system
    public static boolean TRACKING_ENABLED = true;

    // TUNABLE: Use Red goal (true) or Blue goal (false)
    public static boolean USE_RED_GOAL = false;

    // State variables
    private boolean trackingEnabled = true;
    private boolean visionMode = false;
    private long lastTagSeenTime = 0;
    private double targetGlobalHeading = 0.0;
    private boolean poseCalibrated = false;
    private double lastVisionAngle = 0.0; // Store last good vision angle for hold mode
    private double smoothedTurretAngle = 0.0; // Smoothed turret angle for stability

    public CompTurretSystem() {
        addComponents(
                new SubsystemComponent(Turret2.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        // Initialize odometry
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, START_X, START_Y, AngleUnit.DEGREES, START_HEADING));

        // Initialize turret to center position - use raw angle to ensure it's at 240° (straight forward)
        Turret2.INSTANCE.setAngle(0.0); //240.0 <- SetRawAngle

        // Set up FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Configure AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(530.423, 530.423, 447.938, 335.553)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        // Build vision portal with Arducam
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "arducam"))
                .setCameraResolution(new Size(800, 600))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .build();

        dashboard.startCameraStream(visionPortal, 30);

        lastTagSeenTime = System.currentTimeMillis();

        telemetry.addData("Status", "✓ Initialized");
        telemetry.addLine();
        telemetry.addData("Mode", "Hybrid Vision + Odometry Tracking");
        telemetry.addData("Target Tag ID", TARGET_TAG_ID);
        telemetry.addData("Goal", USE_RED_GOAL ? "RED" : "BLUE");
        telemetry.addLine();
        telemetry.addLine("🎮 CONTROLS:");
        telemetry.addData("Left Stick", "Drive + Strafe");
        telemetry.addData("Right Stick X", "Rotate");
        telemetry.addData("A Button", "Toggle Tracking");
        telemetry.addData("Y Button", "Reset to Center");
        telemetry.addData("X Button", "Calibrate Pose (AprilTag)");
        telemetry.addData("DPad Up/Down", "Switch Red/Blue Goal");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // Set up mecanum drive control
        var forward = Gamepads.gamepad1().leftStickY().negate();
        var strafe = Gamepads.gamepad1().leftStickX();
        var rotate = Gamepads.gamepad1().rightStickX();

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

        // A Button: Toggle tracking on/off
        Gamepads.gamepad1().a().whenBecomesTrue(() -> {
            trackingEnabled = !trackingEnabled;
            if (!trackingEnabled) {
                Turret2.INSTANCE.setAngle(0.0);
            }
        });

        // Y Button: Reset turret to center
        Gamepads.gamepad1().y().whenBecomesTrue(() -> {
            trackingEnabled = false;
            Turret2.INSTANCE.setAngle(0.0);
        });

        // X Button: Calibrate pose using AprilTag
        Gamepads.gamepad1().x().whenBecomesTrue(this::calibratePoseFromAprilTag);

        // DPad Up: Switch to Red goal
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(() -> {
            USE_RED_GOAL = true;
        });

        // DPad Down: Switch to Blue goal
        Gamepads.gamepad1().dpadDown().whenBecomesTrue(() -> {
            USE_RED_GOAL = false;
        });
    }

    @Override
    public void onUpdate() {
        // Update odometry (always running in background)
        pinpoint.update();
        Pose2D currentPose = pinpoint.getPosition();

        // DECODE FIELD FIX: Invert X and Y from Pinpoint to match DECODE field coordinate system
        // Pinpoint gives readings in standard FTC field coordinates
        // DECODE field has inverted axes, so we need to flip them for correct visualization
        double currentX = -currentPose.getX(DistanceUnit.INCH);
        double currentY = -currentPose.getY(DistanceUnit.INCH);
        double currentRobotHeading = currentPose.getHeading(AngleUnit.DEGREES);

        // Get current goal position
        double goalX = USE_RED_GOAL ? RED_GOAL_X : BLUE_GOAL_X;
        double goalY = USE_RED_GOAL ? RED_GOAL_Y : BLUE_GOAL_Y;

        // ALWAYS update targetGlobalHeading using atan2 (odometry runs in background every loop)
        targetGlobalHeading = calculateAngleToGoal(currentX, currentY, goalX, goalY);

        // Draw visualization on FTC Dashboard
        drawFieldVisualization(currentX, currentY, currentRobotHeading, goalX, goalY);

        if (!trackingEnabled || !TRACKING_ENABLED) {
            displayManualMode(currentRobotHeading);
            return;
        }

        // Check for AprilTag detections (camera always scanning in background)
        List<AprilTagDetection> detections = aprilTag.getDetections();
        boolean tagDetectedThisFrame = false;
        double tagBearing = 0.0;

        if (!detections.isEmpty()) {
            for (AprilTagDetection tag : detections) {
                if (tag.id == TARGET_TAG_ID) {
                    tagDetectedThisFrame = true;
                    // AprilTag ftcPose.bearing convention:
                    // - POSITIVE bearing = tag is to the RIGHT of camera center
                    // - NEGATIVE bearing = tag is to the LEFT of camera center
                    //
                    // Turret2 ACTUAL PHYSICAL BEHAVIOR (confirmed by testing with Turret2Tuner):
                    // - When you command +45°, turret physically turns RIGHT
                    // - When you command -45°, turret physically turns LEFT
                    //
                    // However, Turret2's INTERNAL coordinate system is INVERTED:
                    // - Turret2.java comment says: "positive = left, negative = right"
                    // - But physically it does the OPPOSITE
                    //
                    // Therefore, to center a tag:
                    // - Tag at +10° RIGHT → Need turret RIGHT → Command would be +10° (but Turret2 inverts it internally)
                    // - We must NEGATE to compensate: -10° gets inverted to +10° physically = RIGHT
                    tagBearing = tag.ftcPose.bearing;
                    lastTagSeenTime = System.currentTimeMillis();
                    break;
                }
            }
        }

        // Calculate how long it's been since we last saw the tag
        double timeSinceLastTag = (System.currentTimeMillis() - lastTagSeenTime) / 1000.0;

        double targetTurretAngle;

        // PRIORITY SYSTEM:
        // - Vision ONLY controls turret when tag is actively detected THIS FRAME
        // - If tag not detected for VISION_TIMEOUT_SEC, odometry takes over
        // - NO dynamic switching - clean handoff

        if (tagDetectedThisFrame) {
            // 🎥 VISION MODE: Tag is visible RIGHT NOW - vision controls turret
            visionMode = true;

            if (Math.abs(tagBearing) > VISION_DEADBAND_DEG) {
                // VISION TRACKING LOGIC:
                // - AprilTag tagBearing: POSITIVE = tag to RIGHT, NEGATIVE = tag to LEFT
                // - Turret2 internal system: POSITIVE = LEFT, NEGATIVE = RIGHT (INVERTED!)
                // - To center tag on RIGHT (+bearing), turret turns RIGHT (NEGATIVE angle)
                // - Therefore: turretCorrection = -tagBearing (NEGATE to match Turret2 coords)

                // Get current turret angle
                double currentTurretAngle = Turret2.INSTANCE.getTargetLogicalDeg();

                // Calculate incremental correction with gain and NEGATION for Turret2's inverted system
                double correction = -tagBearing * VISION_TRACKING_GAIN;

                // Apply correction to current angle
                double desiredAngle = currentTurretAngle + correction;

                // Apply smoothing filter to prevent jitter
                // Initialize smoothedTurretAngle on first vision frame
                if (smoothedTurretAngle == 0.0) {
                    smoothedTurretAngle = currentTurretAngle;
                }

                // Exponential smoothing: blend old and new values
                targetTurretAngle = smoothedTurretAngle * VISION_SMOOTHING + desiredAngle * (1.0 - VISION_SMOOTHING);

                // Clamp to turret physical limits
                targetTurretAngle = Math.max(-Turret2.MAX_ROTATION,
                                            Math.min(Turret2.MAX_ROTATION, targetTurretAngle));

                // Store this as the last good angle for hold mode
                lastVisionAngle = targetTurretAngle;
                smoothedTurretAngle = targetTurretAngle;
            } else {
                // Within deadband - tag is CENTERED, LOCK position
                // Stop moving and hold current angle
                targetTurretAngle = Turret2.INSTANCE.getTargetLogicalDeg();
                lastVisionAngle = targetTurretAngle;
                smoothedTurretAngle = targetTurretAngle;
            }
        } else if (timeSinceLastTag < VISION_TIMEOUT_SEC) {
            // ⏳ HOLD MODE: Tag was recently visible, hold the last good vision angle
            // This prevents the turret from wandering when tag is briefly occluded
            visionMode = true;
            targetTurretAngle = lastVisionAngle; // Hold the last angle we calculated with vision
        } else {
            // 🧭 ODOMETRY MODE: Tag lost for more than VISION_TIMEOUT_SEC
            // Odometry takes over to reacquire the target
            visionMode = false;

            // Use the continuously-updated targetGlobalHeading (from atan2)
            targetTurretAngle = calculateTurretAngle(currentRobotHeading, targetGlobalHeading);
        }

        // Command turret to target angle
        Turret2.INSTANCE.setAngle(targetTurretAngle);

        // Display telemetry
        displayTrackingTelemetry(currentX, currentY, currentRobotHeading, goalX, goalY,
                                 tagDetectedThisFrame, tagBearing, targetTurretAngle, timeSinceLastTag);
    }

    /**
     * Calibrate pinpoint pose using AprilTag detection.
     * This uses the known position of the AprilTag to calculate robot's field position.
     */
    private void calibratePoseFromAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        for (AprilTagDetection tag : detections) {
            if (tag.id == TARGET_TAG_ID && tag.metadata != null) {
                // Get tag's known field position (you may need to adjust these based on your field setup)
                // For tag ID 20, assuming it's at the goal position
                double tagFieldX = USE_RED_GOAL ? RED_GOAL_X : BLUE_GOAL_X;
                double tagFieldY = USE_RED_GOAL ? RED_GOAL_Y : BLUE_GOAL_Y;

                // Get robot's position relative to tag from AprilTag detection
                double rangeToTag = tag.ftcPose.range;
                double bearingToTag = Math.toRadians(tag.ftcPose.bearing);
                double yawToTag = Math.toRadians(tag.ftcPose.yaw);

                // Calculate robot position on field
                // Robot is at (tagX - dx, tagY - dy) where dx/dy are offsets from tag
                double currentHeading = pinpoint.getHeading(AngleUnit.RADIANS);

                // Calculate the vector from robot to tag in field coordinates
                double globalBearing = currentHeading + bearingToTag;
                double dx = rangeToTag * Math.cos(globalBearing);
                double dy = rangeToTag * Math.sin(globalBearing);

                // Robot position is tag position minus the offset
                double robotX = tagFieldX - dx;
                double robotY = tagFieldY - dy;

                // Update pinpoint position (keep current heading from IMU as it's more accurate)
                double currentHeadingDeg = pinpoint.getHeading(AngleUnit.DEGREES);
                pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, robotX, robotY,
                                               AngleUnit.DEGREES, currentHeadingDeg));

                poseCalibrated = true;

                // Rumble to indicate successful calibration
                Gamepads.gamepad1().getGamepad().invoke().rumble(200);
                break;
            }
        }
    }

    /**
     * Draw field visualization on FTC Dashboard
     */
    private void drawFieldVisualization(double currentX, double currentY, double currentRobotHeading,
                                        double goalX, double goalY) {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        // Draw goal position (red X marker for red goal, blue for blue goal)
        String goalColor = USE_RED_GOAL ? "#FF0000" : "#0000FF";
        fieldOverlay.setStroke(goalColor);
        fieldOverlay.setStrokeWidth(2);
        double goalSize = 4;
        fieldOverlay.strokeLine(goalX - goalSize, goalY - goalSize, goalX + goalSize, goalY + goalSize);
        fieldOverlay.strokeLine(goalX - goalSize, goalY + goalSize, goalX + goalSize, goalY - goalSize);
        fieldOverlay.strokeCircle(goalX, goalY, 6);

        // Draw robot position (blue circle with heading indicator)
        fieldOverlay.setStroke("#0000FF");
        fieldOverlay.setFill("#0000FF");
        fieldOverlay.fillCircle(currentX, currentY, 6);

        // Draw robot heading arrow (blue)
        double headingRadians = Math.toRadians(currentRobotHeading);
        double arrowLength = 12;
        double arrowEndX = currentX + arrowLength * Math.cos(headingRadians);
        double arrowEndY = currentY + arrowLength * Math.sin(headingRadians);
        fieldOverlay.strokeLine(currentX, currentY, arrowEndX, arrowEndY);

        // Draw line from robot to goal (green line)
        fieldOverlay.setStroke("#00FF00");
        fieldOverlay.setStrokeWidth(1);
        fieldOverlay.strokeLine(currentX, currentY, goalX, goalY);

        // Draw turret direction (yellow line)
        if (trackingEnabled) {
            // Turret (ACTUAL TESTED): NEGATIVE = LEFT, POSITIVE = RIGHT
            // To get global heading from turret angle:
            // - Negative turret (left) = CCW from robot heading = ADD to heading (but turret is negative, so subtract)
            // - Positive turret (right) = CW from robot heading = SUBTRACT from heading (but turret is positive, so subtract)
            // So: globalTurretHeading = robotHeading - turretAngle

            double turretLogical = Turret2.INSTANCE.getTargetLogicalDeg();
            double turretGlobalHeading = currentRobotHeading - turretLogical;
            double turretRadians = Math.toRadians(turretGlobalHeading);
            double turretLength = 18;
            double turretEndX = currentX + turretLength * Math.cos(turretRadians);
            double turretEndY = currentY + turretLength * Math.sin(turretRadians);

            // Yellow for odometry mode, Orange for vision mode
            fieldOverlay.setStroke(visionMode ? "#FFA500" : "#FFFF00");
            fieldOverlay.setStrokeWidth(3);
            fieldOverlay.strokeLine(currentX, currentY, turretEndX, turretEndY);
        }

        // Send packet to dashboard
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    /**
     * Calculate the angle from current position to goal position using atan2.
     */
    private double calculateAngleToGoal(double currentX, double currentY, double goalX, double goalY) {
        double deltaX = goalX - currentX;
        double deltaY = goalY - currentY;

        double angleRad = Math.atan2(deltaY, deltaX);
        double angleDeg = Math.toDegrees(angleRad);

        return normalizeAngle(angleDeg);
    }

    /**
     * Calculate the turret angle needed to point at the target global heading.
     *
     * COORDINATE SYSTEMS:
     * - Field: atan2 gives angle where 0° = +X axis, 90° = +Y axis (CCW positive)
     * - Robot heading: 0° = +X axis, 90° = +Y axis (CCW positive)
     * - Turret (ACTUAL TESTED): 0° = forward, NEGATIVE = LEFT, POSITIVE = RIGHT
     *
     * EXAMPLE:
     * - Robot at (0,0) facing +45° (toward upper-right on dashboard)
     * - Blue Goal at (0, 72) - directly UP on dashboard
     * - globalTarget = atan2(72, 0) = 90° (pointing up, toward +Y)
     * - angleDiff = 90° - 45° = 45° (goal is 45° CCW from robot = to the LEFT)
     * - Turret should turn LEFT = NEGATIVE angle = -45°
     * - So turretAngle = -angleDiff = -45° ✓
     */
    private double calculateTurretAngle(double robotHeading, double globalTarget) {
        double angleDiff = globalTarget - robotHeading;

        // Normalize to [-180, 180] range
        angleDiff = normalizeAngleSigned(angleDiff);

        // angleDiff: positive = target is CCW from robot heading (LEFT)
        // angleDiff: negative = target is CW from robot heading (RIGHT)
        // Turret (TESTED): NEGATIVE = LEFT, POSITIVE = RIGHT
        // These are OPPOSITE, so we negate: turretAngle = -angleDiff
        double logicalTurretAngle = -angleDiff;

        // Clamp to turret limits
        return Math.max(-Turret2.MAX_ROTATION, Math.min(Turret2.MAX_ROTATION, logicalTurretAngle));
    }

    private void displayTrackingTelemetry(double robotX, double robotY, double robotHeading,
                                          double goalX, double goalY,
                                          boolean tagDetected, double tagBearing, double targetAngle, double timeSinceLastTag) {
        telemetry.addLine("═══ COMP TURRET SYSTEM ═══");
        telemetry.addLine();

        // Show priority system status with timeout
        telemetry.addLine("📋 PRIORITY TRACKING SYSTEM:");
        if (visionMode) {
            telemetry.addData("  🎯 Vision Priority", "ACTIVE (Controls Turret)");
            telemetry.addData("  🧭 Odometry", "Standby (Running in background)");
            if (!tagDetected) {
                double timeUntilOdom = VISION_TIMEOUT_SEC - timeSinceLastTag;
                telemetry.addData("  ⏱️ Odometry takeover in", "%.1f sec", timeUntilOdom);
            }
        } else {
            telemetry.addData("  🧭 Odometry Priority", "ACTIVE (Controls Turret)");
            telemetry.addData("  🎯 Vision", "Scanning (Tag lost %.1fs ago)", timeSinceLastTag);
        }
        telemetry.addLine();

        // Mode indicator
        if (visionMode) {
            telemetry.addLine("🎯 MODE: VISION LOCK");
            telemetry.addData("  Tag ID", TARGET_TAG_ID);
            telemetry.addData("  Tag Detected", tagDetected ? "✓ YES" : "✗ NO (holding)");
            if (tagDetected) {
                telemetry.addData("  Tag Bearing", "%.2f°", tagBearing);
                telemetry.addData("  Status", Math.abs(tagBearing) < VISION_DEADBAND_DEG ? "✓ CENTERED" : "⟳ CENTERING");
            }
        } else {
            telemetry.addLine("🧭 MODE: ODOMETRY ACQUISITION");
            telemetry.addData("  Goal", USE_RED_GOAL ? "RED" : "BLUE");
            telemetry.addData("  Goal Position", "(%.1f, %.1f)", goalX, goalY);
            telemetry.addData("  Bearing to Goal", "%.1f°", targetGlobalHeading);
            telemetry.addData("  Status", "Searching for tag...");
        }
        telemetry.addLine();

        // Robot state
        telemetry.addLine("🤖 ROBOT");
        telemetry.addData("  Position", "(%.1f, %.1f)", robotX, robotY);
        telemetry.addData("  Heading", "%.1f°", robotHeading);
        telemetry.addData("  Pose Calibrated", poseCalibrated ? "✓ YES" : "✗ NO");
        telemetry.addLine();

        // Turret state
        telemetry.addLine("🔄 TURRET");
        telemetry.addData("  Logical Angle", "%.1f°", Turret2.INSTANCE.getTargetLogicalDeg());
        telemetry.addData("  Raw Angle", "%.1f°", Turret2.INSTANCE.getCurrentRawDeg());
        telemetry.addData("  Servo Position", "%.3f", Turret2.INSTANCE.getServoPosition());
        telemetry.addData("  Range", "±%.0f°", Turret2.MAX_ROTATION);
        telemetry.addLine();

        // Check if at limits
        boolean atMinLimit = Math.abs(Turret2.INSTANCE.getTargetLogicalDeg() + Turret2.MAX_ROTATION) < 1.0;
        boolean atMaxLimit = Math.abs(Turret2.INSTANCE.getTargetLogicalDeg() - Turret2.MAX_ROTATION) < 1.0;

        if (atMinLimit || atMaxLimit) {
            telemetry.addLine("⚠️ WARNING: TURRET AT LIMIT");
            telemetry.addLine();
        }

        // Controls
        telemetry.addLine("🎮 CONTROLS:");
        telemetry.addData("  A", trackingEnabled ? "Disable Tracking" : "Enable Tracking");
        telemetry.addData("  Y", "Reset to Center");
        telemetry.addData("  X", "Calibrate Pose");
        telemetry.addData("  DPad ↑↓", "Switch Goal");
        telemetry.addLine();

        // Dashboard tuning reminder
        telemetry.addLine("💡 DASHBOARD SETTINGS:");
        telemetry.addData("  VISION_TIMEOUT_SEC", "%.1f sec", VISION_TIMEOUT_SEC);
        telemetry.addData("  VISION_TRACKING_GAIN", "%.2f", VISION_TRACKING_GAIN);
        telemetry.addData("  VISION_DEADBAND_DEG", "%.1f°", VISION_DEADBAND_DEG);

        telemetry.update();
    }

    private void displayManualMode(double robotHeading) {
        telemetry.addLine("═══ MANUAL MODE ═══");
        telemetry.addLine();
        telemetry.addData("Tracking", "❌ DISABLED");
        telemetry.addLine();
        telemetry.addData("Turret Logical", "%.1f°", Turret2.INSTANCE.getTargetLogicalDeg());
        telemetry.addData("Turret Raw", "%.1f°", Turret2.INSTANCE.getCurrentRawDeg());
        telemetry.addData("Robot Heading", "%.1f°", robotHeading);
        telemetry.addLine();
        telemetry.addData("Press A", "to enable tracking");
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
        // Return turret to safe center position
        Turret2.INSTANCE.setAngle(0.0);

        // Close vision portal
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}

