package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.teamcode.constants.AutoPoseMemory;
import org.firstinspires.ftc.teamcode.constants.PedroConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret2;

@Autonomous(name = "BlueTrackingAuto")
public class BlueTrackingAuto extends NextFTCOpMode {

    // Start pose defined in traditional FTC coordinates.
    public static double START_TRAD_X = 0.0;
    public static double START_TRAD_Y = 0.0;
    public static double START_TRAD_HEADING = 270.0;

    // Blue goal in Decode coordinates (same convention used in teleop tracking).
    public static double BLUE_GOAL_X = -72.0;
    public static double BLUE_GOAL_Y = -72.0;

    private Follower follower;

    private double lastPedroX = 0.0;
    private double lastPedroY = 0.0;
    private double lastPedroHeadingDeg = 0.0;
    private double lastTraditionalHeadingDeg = 0.0;
    private double lastFtcX = 0.0;
    private double lastFtcY = 0.0;
    private double targetGlobalHeading = 0.0;
    private double targetTurretAngle = 0.0;
    private boolean trackingEnabled = true;

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

    @Override
    public void onInit() {
        // Always reset carryover memory at the start of this auto run.
        AutoPoseMemory.clear();

        double startPedroX = AutoPoseMemory.traditionalToPedroX(START_TRAD_X, START_TRAD_Y);
        double startPedroY = AutoPoseMemory.traditionalToPedroY(START_TRAD_X, START_TRAD_Y);
        double startPedroHeading = AutoPoseMemory.traditionalToPedroHeading(START_TRAD_HEADING);

        follower = PedroComponent.follower();
        follower.setStartingPose(
                new Pose(startPedroX, startPedroY, Math.toRadians(startPedroHeading))
        );
        Turret2.INSTANCE.setAngle(0.0);
    }

    @Override
    public void onStartButtonPressed() {
        // Intentionally no movement. Robot is hand-dragged; dead wheels track pose.
    }

    @Override
    public void onUpdate() {
        updatePose();
        if (trackingEnabled) {
            updateTurretTracking();
        }
        AutoPoseMemory.setFtcPose(lastFtcX, lastFtcY, lastTraditionalHeadingDeg);

        telemetry.addData("Mode", "TRACK ONLY (no drive commands)");
        telemetry.addData("Pose Pedro (x,y,hdg)", "(%.1f, %.1f, %.1f°)", lastPedroX, lastPedroY, lastPedroHeadingDeg);
        telemetry.addData("Pose FTC (x,y,hdg)", "(%.1f, %.1f, %.1f°)", lastFtcX, lastFtcY, lastTraditionalHeadingDeg);
        telemetry.addData("Turret Track", trackingEnabled ? "ON" : "OFF");
        telemetry.addData("Goal Decode (x,y)", "(%.1f, %.1f)", BLUE_GOAL_X, BLUE_GOAL_Y);
        telemetry.addData("Target Global Hdg", "%.1f°", targetGlobalHeading);
        telemetry.addData("Target Turret", "%.1f°", targetTurretAngle);
        telemetry.addData("AutoPoseMemory", "has=%s (%.1f, %.1f, %.1f°)",
                AutoPoseMemory.hasPose, AutoPoseMemory.ftcX, AutoPoseMemory.ftcY, AutoPoseMemory.headingDeg);
        telemetry.update();
    }

    private void updatePose() {
        Pose pose = follower.getPose();
        lastPedroX = pose.getX();
        lastPedroY = pose.getY();
        lastPedroHeadingDeg = normalizeAngle(Math.toDegrees(pose.getHeading()));
        lastTraditionalHeadingDeg = AutoPoseMemory.pedroToTraditionalHeading(lastPedroHeadingDeg);

        lastFtcX = AutoPoseMemory.pedroToTraditionalX(lastPedroX, lastPedroY);
        lastFtcY = AutoPoseMemory.pedroToTraditionalY(lastPedroX, lastPedroY);
    }

    private void updateTurretTracking() {
        // Convert FTC-centered pose to Decode coordinates.
        double currentDecodeX = -lastFtcX;
        double currentDecodeY = -lastFtcY;

        targetGlobalHeading = calculateAngleToGoal(currentDecodeX, currentDecodeY, BLUE_GOAL_X, BLUE_GOAL_Y);
        targetTurretAngle = calculateTurretAngle(lastTraditionalHeadingDeg, targetGlobalHeading);
        Turret2.INSTANCE.setAngle(targetTurretAngle);
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

    private double calculateAngleToGoal(double currentX, double currentY, double goalX, double goalY) {
        double deltaX = goalX - currentX;
        double deltaY = goalY - currentY;
        double angleRad = Math.atan2(deltaY, deltaX);
        return normalizeAngle(Math.toDegrees(angleRad));
    }

    private double calculateTurretAngle(double robotHeading, double globalTarget) {
        double angleDiff = globalTarget - robotHeading;
        angleDiff = normalizeAngleSigned(angleDiff);
        double logicalTurretAngle = -angleDiff;
        return Math.max(-Turret2.MAX_ROTATION, Math.min(Turret2.MAX_ROTATION, logicalTurretAngle));
    }

    @Override
    public void onStop() {
        updatePose();
        AutoPoseMemory.setFtcPose(lastFtcX, lastFtcY, lastTraditionalHeadingDeg);
        Turret2.INSTANCE.setAngle(0.0);
    }
}
