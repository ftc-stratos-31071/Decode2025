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

    // Requested Pedro start pose for this tracking opmode.
    public static double PEDRO_START_X = 72.0;
    public static double PEDRO_START_Y = 48.0;
    public static double PEDRO_START_HEADING = 180.0;

    private Follower follower;

    private double lastPedroX = 0.0;
    private double lastPedroY = 0.0;
    private double lastPedroHeadingDeg = 0.0;
    private double lastTraditionalHeadingDeg = 0.0;
    private double lastFtcX = 0.0;
    private double lastFtcY = 0.0;

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

        follower = PedroComponent.follower();
        follower.setStartingPose(
                new Pose(PEDRO_START_X, PEDRO_START_Y, Math.toRadians(PEDRO_START_HEADING))
        );
    }

    @Override
    public void onStartButtonPressed() {
        // Intentionally no movement. Robot is hand-dragged; dead wheels track pose.
    }

    @Override
    public void onUpdate() {
        updatePose();
        AutoPoseMemory.setFtcPose(lastFtcX, lastFtcY, lastTraditionalHeadingDeg);

        telemetry.addData("Mode", "TRACK ONLY (no drive commands)");
        telemetry.addData("Pose Pedro (x,y,hdg)", "(%.1f, %.1f, %.1f°)", lastPedroX, lastPedroY, lastPedroHeadingDeg);
        telemetry.addData("Pose FTC (x,y,hdg)", "(%.1f, %.1f, %.1f°)", lastFtcX, lastFtcY, lastTraditionalHeadingDeg);
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

    private double normalizeAngle(double degrees) {
        degrees = degrees % 360.0;
        if (degrees < 0) degrees += 360.0;
        return degrees;
    }

    @Override
    public void onStop() {
        updatePose();
        AutoPoseMemory.setFtcPose(lastFtcX, lastFtcY, lastTraditionalHeadingDeg);
    }
}
