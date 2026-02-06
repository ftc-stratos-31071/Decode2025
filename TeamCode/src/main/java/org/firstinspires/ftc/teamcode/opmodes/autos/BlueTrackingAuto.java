package org.firstinspires.ftc.teamcode.opmodes.autos;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.List;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.PedroConstants;
import org.firstinspires.ftc.teamcode.opmodes.teleops.BlueTeleop;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret2;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "BlueTrackingAuto")
public class BlueTrackingAuto extends NextFTCOpMode {

    private Follower follower;
    private PathChain strafePath;

    // Match FarBlueAuto start pose
    public static double START_X = 56.0;
    public static double START_Y = 8.0;
    public static double START_HEADING = 180.0;

    public static double STRAFE_DISTANCE = 10.0;

    // Tracking settings (same goal as BlueTeleop, Decode coordinates)
    public static int TARGET_TAG_ID = 20;
    public static double BLUE_GOAL_X = -72.0;
    public static double BLUE_GOAL_Y = -72.0;

    public static double VISION_TRACKING_GAIN = 0.3;
    public static double VISION_TIMEOUT_SEC = 0.5;
    public static double VISION_DEADBAND_DEG = 5.0;
    public static double VISION_SMOOTHING = 0.3;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private boolean trackingEnabled = true;
    private boolean visionMode = false;
    private long lastTagSeenTime = 0;
    private double targetGlobalHeading = 0.0;
    private double lastVisionAngle = 0.0;
    private double smoothedTurretAngle = 0.0;

    private double lastPedroX = 0.0;
    private double lastPedroY = 0.0;
    private double lastHeadingDeg = 0.0;
    private double lastFtcX = 0.0;
    private double lastFtcY = 0.0;
    private double lastDecodeX = 0.0;
    private double lastDecodeY = 0.0;

    public BlueTrackingAuto() {
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
        double endX = START_X;
        double endY = START_Y + STRAFE_DISTANCE;

        strafePath = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(START_X, START_Y),
                                new Pose(endX, endY)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(START_HEADING), Math.toRadians(START_HEADING))
                .build();
    }

    @Override
    public void onInit() {
        follower = PedroComponent.follower();
        follower.setStartingPose(
                new Pose(START_X, START_Y, Math.toRadians(START_HEADING))
        );
        buildPaths();

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

        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);
        lastTagSeenTime = System.currentTimeMillis();
    }

    @Override
    public void onStartButtonPressed() {
        new FollowPath(strafePath).schedule();
    }

    @Override
    public void onUpdate() {
        updatePose();

        if (trackingEnabled) {
            updateTurretTracking();
        }

        telemetry.addData("Pose Pedro (x,y,hdg)", "(%.1f, %.1f, %.1f°)", lastPedroX, lastPedroY, lastHeadingDeg);
        telemetry.addData("Pose FTC (x,y,hdg)", "(%.1f, %.1f, %.1f°)", lastFtcX, lastFtcY, lastHeadingDeg);
        telemetry.addData("Pose Decode (x,y,hdg)", "(%.1f, %.1f, %.1f°)", lastDecodeX, lastDecodeY, lastHeadingDeg);
        telemetry.update();
    }

    private void updatePose() {
        Pose pose = follower.getPose();
        lastPedroX = pose.getX();
        lastPedroY = pose.getY();
        lastHeadingDeg = Math.toDegrees(pose.getHeading());

        // Map Pedro field coords to dashboard/traditional FTC-centered coords.
        // Requested mapping examples:
        // Pedro (0,144)   -> FTC (-72,-72)
        // Pedro (144,144) -> FTC (-72, 72)
        lastFtcX = 72.0 - lastPedroY;
        lastFtcY = lastPedroX - 72.0;
        lastDecodeX = -lastFtcX;
        lastDecodeY = -lastFtcY;
    }

    private void updateTurretTracking() {
        double currentX = lastDecodeX;
        double currentY = lastDecodeY;
        double currentRobotHeading = lastHeadingDeg;

        double goalX = BLUE_GOAL_X;
        double goalY = BLUE_GOAL_Y;

        targetGlobalHeading = calculateAngleToGoal(currentX, currentY, goalX, goalY);

        double dashboardGoalX = -goalX;
        double dashboardGoalY = -goalY;
        drawFieldVisualization(lastFtcX, lastFtcY, currentRobotHeading, dashboardGoalX, dashboardGoalY);

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

        double timeSinceLastTag = (System.currentTimeMillis() - lastTagSeenTime) / 1000.0;

        double targetTurretAngle;

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
        } else if (timeSinceLastTag < VISION_TIMEOUT_SEC) {
            visionMode = true;
            targetTurretAngle = lastVisionAngle;
        } else {
            visionMode = false;
            targetTurretAngle = calculateTurretAngle(currentRobotHeading, targetGlobalHeading);
        }

        Turret2.INSTANCE.setAngle(targetTurretAngle);
    }

    private void drawFieldVisualization(double currentX, double currentY, double currentRobotHeading,
                                        double goalX, double goalY) {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        String goalColor = "#0000FF";
        fieldOverlay.setStroke(goalColor);
        fieldOverlay.setStrokeWidth(2);
        double goalSize = 4;
        fieldOverlay.strokeLine(goalX - goalSize, goalY - goalSize, goalX + goalSize, goalY + goalSize);
        fieldOverlay.strokeLine(goalX - goalSize, goalY + goalSize, goalX + goalSize, goalY - goalSize);
        fieldOverlay.strokeCircle(goalX, goalY, 6);

        fieldOverlay.setStroke("#0000FF");
        fieldOverlay.setFill("#0000FF");
        fieldOverlay.fillCircle(currentX, currentY, 6);

        double headingRadians = Math.toRadians(currentRobotHeading);
        double arrowLength = 12;
        double arrowEndX = currentX + arrowLength * Math.cos(headingRadians);
        double arrowEndY = currentY + arrowLength * Math.sin(headingRadians);
        fieldOverlay.strokeLine(currentX, currentY, arrowEndX, arrowEndY);

        fieldOverlay.setStroke("#00FF00");
        fieldOverlay.setStrokeWidth(1);
        fieldOverlay.strokeLine(currentX, currentY, goalX, goalY);

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
        Pose finalPose = follower.getPose();
        double finalFtcX = finalPose.getX() - 72.0;
        double finalFtcY = finalPose.getY() - 72.0;
        double finalHeadingDeg = normalizeAngle(Math.toDegrees(finalPose.getHeading()));

        BlueTeleop.AUTO_START_X = finalFtcX;
        BlueTeleop.AUTO_START_Y = finalFtcY;
        BlueTeleop.AUTO_START_HEADING = finalHeadingDeg;
        BlueTeleop.USE_AUTO_START_POSE = true;

        Turret2.INSTANCE.setAngle(0.0);
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
