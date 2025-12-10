package org.firstinspires.ftc.teamcode.roadrunner.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import dev.nextftc.ftc.NextFTCOpMode;

/**
 * Example autonomous OpMode using NextFTC with RoadRunner.
 * This demonstrates basic trajectory following using the action builder.
 */
@Autonomous(name = "RoadRunner Auto Example", group = "Examples")
public class RoadRunnerAutoExample extends NextFTCOpMode {
    private final Pose2d startPose = new Pose2d(0, 0, 0);
    private final Pose2d scorePose = new Pose2d(30, 30, Math.toRadians(90));

    private MecanumDrive drive;
    private Action trajectoryAction;
    private boolean actionRunning = false;

    @Override
    public void onInit() {
        drive = new MecanumDrive(hardwareMap, startPose);

        // Build trajectory action using RoadRunner's action builder
        trajectoryAction = drive.actionBuilder(startPose)
                .splineTo(scorePose.position, scorePose.heading)
                .build();
    }

    @Override
    public void onStartButtonPressed() {
        actionRunning = true;
    }

    @Override
    public void onUpdate() {
        // Run the action if it's active
        if (actionRunning) {
            TelemetryPacket packet = new TelemetryPacket();
            actionRunning = trajectoryAction.run(packet);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        // Display telemetry
        telemetry.addData("x", drive.localizer.getPose().position.x);
        telemetry.addData("y", drive.localizer.getPose().position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
        telemetry.addData("Action Running", actionRunning);
        telemetry.update();
    }
}
