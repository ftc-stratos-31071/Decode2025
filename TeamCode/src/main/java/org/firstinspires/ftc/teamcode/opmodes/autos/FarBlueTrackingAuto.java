package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.subsystems.Shooter.*;

import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.commands.AutoDriveTimeoutCmd;
import org.firstinspires.ftc.teamcode.commands.IntakeSeqCmd;
import org.firstinspires.ftc.teamcode.commands.RapidFireTimeoutCmd;
import org.firstinspires.ftc.teamcode.commands.WaitCmd;
import org.firstinspires.ftc.teamcode.constants.PedroConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret2;
import org.firstinspires.ftc.teamcode.opmodes.teleops.BlueTeleop;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "FarBlueTrackingAuto")
public class FarBlueTrackingAuto extends NextFTCOpMode {

    private Follower follower;

    private PathChain path1;
    private PathChain path2;
    private PathChain path3;
    private PathChain path4;
    private PathChain path5;
    private PathChain path6;
    private PathChain path7;

    // Tracking settings (from CompTurretSystem / BlueTeleop)
    public static int TARGET_TAG_ID = 20;
    public static double BLUE_GOAL_X = -72.0;
    public static double BLUE_GOAL_Y = -72.0;
    public static double START_X = 0.0;
    public static double START_Y = 0.0;
    public static double START_HEADING = 90.0;

    public static double VISION_TRACKING_GAIN = 0.3;
    public static double VISION_TIMEOUT_SEC = 0.5;
    public static double VISION_DEADBAND_DEG = 5.0;
    public static double VISION_SMOOTHING = 0.3;

    private GoBildaPinpointDriver pinpoint;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private boolean trackingEnabled = true;
    private boolean visionMode = false;
    private long lastTagSeenTime = 0;
    private double targetGlobalHeading = 0.0;
    private double lastVisionAngle = 0.0;
    private double smoothedTurretAngle = 0.0;

    public FarBlueTrackingAuto() {
        addComponents(
                new SubsystemComponent(
                        Intake.INSTANCE,
                        Shooter.INSTANCE,
                        Turret2.INSTANCE
                ),
                new PedroComponent(PedroConstants::createFollower),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    public void buildPaths() {

        path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 8.000),

                                new Pose(3, 8.322)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(3, 8.322),
                                new Pose(56.000, 8)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();


        path3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(56.000, 8),
                                new Pose(48.548, 38.514),
                                new Pose(18.597, 35.774)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                                new Pose(18.597, 35.774),
                                new Pose(21.990, 33.279),
                                new Pose(25.383, 30.784),
                                new Pose(28.776, 28.289),
                                new Pose(32.169, 25.794),
                                new Pose(35.561, 23.299),
                                new Pose(38.954, 20.804),
                                new Pose(42.347, 18.309),
                                new Pose(45.740, 15.813),
                                new Pose(49.132, 13.318),
                                new Pose(56, 8)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56, 8),
                                new Pose(3, 27)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(3, 27),
                                new Pose(56, 8)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56, 8),
                                new Pose(36, 8)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    @Override
    public void onInit() {
        // CRITICAL: Reset all subsystem state from previous runs
        Shooter.INSTANCE.stop();
        Intake.INSTANCE.setIntakePower(0.0);
        Intake.INSTANCE.setTransferPower(0.0);

        // Reset hood to lower angle for far shots
        Shooter.INSTANCE.setHood(ShooterConstants.farHoodPos).schedule();

        // Close door on init
        Intake.INSTANCE.moveServoPos().schedule();

        // Reset turret angle
        Turret2.INSTANCE.setAngle(0.0);
        Turret2.INSTANCE.goToAngle(0.0);
        Shooter.INSTANCE.stop();

        // Initialize odometry for tracking
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, START_X, START_Y, AngleUnit.DEGREES, START_HEADING));

        // Configure AprilTag processor
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

        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);
        lastTagSeenTime = System.currentTimeMillis();

        follower = PedroConstants.createFollower(hardwareMap);
        follower.setStartingPose(
                new Pose(56.000, 8.000, Math.toRadians(180))
        );
        buildPaths();
    }

    @Override
    public void onStartButtonPressed() {
        Shooter.INSTANCE.runRPM(ShooterConstants.farTargetRPM).schedule();

        new SequentialGroup(
                Turret2.INSTANCE.goToAngle(85.0),
                Intake.INSTANCE.defaultPos(),
                WaitCmd.create(0.25),
                RapidFireTimeoutCmd.create(1500),
                Intake.INSTANCE.moveServoPos(),
                IntakeSeqCmd.create(),
                AutoDriveTimeoutCmd.create(new FollowPath(path1),2),
                new FollowPath(path2),
                Intake.INSTANCE.zeroPowerIntake(),
                Intake.INSTANCE.zeroPowerTransfer(),
////
                // SHOOT collected balls
                Intake.INSTANCE.defaultPos(),
                WaitCmd.create(0.25),
                RapidFireTimeoutCmd.create(1500),
//
//                // Cycle 2: Drive and collect
                Intake.INSTANCE.moveServoPos(),
                IntakeSeqCmd.create(),
                new FollowPath(path3),
                new FollowPath(path4),
                Intake.INSTANCE.zeroPowerIntake(),
                Intake.INSTANCE.zeroPowerTransfer(),
//
//                // SHOOT
                Intake.INSTANCE.defaultPos(),
                WaitCmd.create(0.25),
                RapidFireTimeoutCmd.create(1500)
//
//                // Cycle 3: Drive and collect
//                Intake.INSTANCE.moveServoPos(),
//                IntakeSeqCmd.create(),
//                AutoDriveTimeoutCmd.create(new FollowPath(path5),2),
//                new FollowPath(path6),
//                Intake.INSTANCE.zeroPowerIntake(),
//                Intake.INSTANCE.zeroPowerTransfer(),
//
//                // SHOOT
//                Intake.INSTANCE.defaultPos(),
//                RapidFireTimeoutCmd.create(3000),
//
//                // Cycle 4: Drive and collect
//                Intake.INSTANCE.moveServoPos(),
//                IntakeSeqCmd.create(),
//                AutoDriveTimeoutCmd.create(new FollowPath(path1),2),
//                new FollowPath(path2),
//                Intake.INSTANCE.zeroPowerIntake(),
//                Intake.INSTANCE.zeroPowerTransfer(),
//
//                // SHOOT
//                Intake.INSTANCE.defaultPos(),
//                RapidFireTimeoutCmd.create(3000),
//
//                // Cycle 5: Drive and collect
//                Intake.INSTANCE.moveServoPos(),
//                IntakeSeqCmd.create(),
//                AutoDriveTimeoutCmd.create(new FollowPath(path1),2),
//                new FollowPath(path2),
//                Intake.INSTANCE.zeroPowerIntake(),
//                Intake.INSTANCE.zeroPowerTransfer(),
//
//                // FINAL SHOOT
//                Intake.INSTANCE.defaultPos(),
//                RapidFireTimeoutCmd.create(3000),
//
//                // Final positioning
//                new FollowPath(path7)
        ).invoke();
    }

    @Override
    public void onUpdate() {
        if (trackingEnabled) {
            updateTurretTracking();
        }
        updateBlueTeleopStartPose();
    }

    private void updateBlueTeleopStartPose() {
        Pose2D pose = pinpoint.getPosition();
        BlueTeleop.AUTO_START_X = pose.getX(DistanceUnit.INCH);
        BlueTeleop.AUTO_START_Y = pose.getY(DistanceUnit.INCH);
        BlueTeleop.AUTO_START_HEADING = pose.getHeading(AngleUnit.DEGREES);
        BlueTeleop.USE_AUTO_START_POSE = true;
    }

    /**
     * Update turret tracking using hybrid odometry + vision system
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
        double goalX = BLUE_GOAL_X;
        double goalY = BLUE_GOAL_Y;

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

        double targetTurretAngle;

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
            // ODOMETRY MODE: Tag lost, use odometry
            visionMode = false;
            targetTurretAngle = calculateTurretAngle(currentRobotHeading, targetGlobalHeading);
        }

        // Command turret to target angle
        Turret2.INSTANCE.setAngle(targetTurretAngle);
    }

    private void drawFieldVisualization(double currentX, double currentY, double currentRobotHeading,
                                        double goalX, double goalY) {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        // Draw goal
        String goalColor = "#0000FF";
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

    private double calculateAngleToGoal(double currentX, double currentY, double goalX, double goalY) {
        double deltaX = goalX - currentX;
        double deltaY = goalY - currentY;
        double angleRad = Math.atan2(deltaY, deltaX);
        double angleDeg = Math.toDegrees(angleRad);
        return normalizeAngle(angleDeg);
    }

    private double calculateTurretAngle(double robotHeading, double globalTarget) {
        double angleDiff = globalTarget - robotHeading;
        angleDiff = normalizeAngleSigned(angleDiff);
        double logicalTurretAngle = -angleDiff;
        return Math.max(-Turret2.MAX_ROTATION, Math.min(Turret2.MAX_ROTATION, logicalTurretAngle));
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
