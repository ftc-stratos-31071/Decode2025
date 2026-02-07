package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
    private double lastHeadingDeg = 0.0;
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
        AutoPoseMemory.setFtcPose(lastFtcX, lastFtcY, lastHeadingDeg);
        drawFieldVisualization(lastFtcX, lastFtcY, lastHeadingDeg);

        telemetry.addData("Mode", "TRACK ONLY (no drive commands)");
        telemetry.addData("Pose Pedro (x,y,hdg)", "(%.1f, %.1f, %.1f°)", lastPedroX, lastPedroY, lastHeadingDeg);
        telemetry.addData("Pose FTC (x,y,hdg)", "(%.1f, %.1f, %.1f°)", lastFtcX, lastFtcY, lastHeadingDeg);
        telemetry.addData("AutoPoseMemory", "has=%s (%.1f, %.1f, %.1f°)",
                AutoPoseMemory.hasPose, AutoPoseMemory.ftcX, AutoPoseMemory.ftcY, AutoPoseMemory.headingDeg);
        telemetry.update();
    }

    private void updatePose() {
        Pose pose = follower.getPose();
        lastPedroX = pose.getX();
        lastPedroY = pose.getY();
        lastHeadingDeg = normalizeAngle(Math.toDegrees(pose.getHeading()));

        lastFtcX = AutoPoseMemory.pedroToTraditionalX(lastPedroX, lastPedroY);
        lastFtcY = AutoPoseMemory.pedroToTraditionalY(lastPedroX, lastPedroY);
    }

    private void drawFieldVisualization(double ftcX, double ftcY, double headingDeg) {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        fieldOverlay.setStroke("#00A8FF");
        fieldOverlay.setFill("#00A8FF");
        fieldOverlay.fillCircle(ftcX, ftcY, 6);

        double headingRadians = Math.toRadians(headingDeg);
        double arrowLength = 12;
        double arrowEndX = ftcX + arrowLength * Math.cos(headingRadians);
        double arrowEndY = ftcY + arrowLength * Math.sin(headingRadians);
        fieldOverlay.setStrokeWidth(2);
        fieldOverlay.strokeLine(ftcX, ftcY, arrowEndX, arrowEndY);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    private double normalizeAngle(double degrees) {
        degrees = degrees % 360.0;
        if (degrees < 0) degrees += 360.0;
        return degrees;
    }

    @Override
    public void onStop() {
        updatePose();
        AutoPoseMemory.setFtcPose(lastFtcX, lastFtcY, lastHeadingDeg);
    }
}
