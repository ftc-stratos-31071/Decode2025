package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            Action trajectoryAction = drive.actionBuilder(beginPose)
                    .splineTo(new Vector2d(30, 30), Math.PI / 2)
                    .splineTo(new Vector2d(0, 60), Math.PI)
                    .build();

            waitForStart();

            while (opModeIsActive()) {
                TelemetryPacket packet = new TelemetryPacket();
                trajectoryAction.preview(packet.fieldOverlay());
                FtcDashboard.getInstance().sendTelemetryPacket(packet);

                Actions.runBlocking(trajectoryAction);

                // Reset to beginning
                drive.localizer.setPose(beginPose);
            }
        } else {
            throw new RuntimeException();
        }
    }
}
