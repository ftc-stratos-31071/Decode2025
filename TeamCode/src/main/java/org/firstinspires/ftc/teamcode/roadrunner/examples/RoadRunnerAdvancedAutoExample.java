package org.firstinspires.ftc.teamcode.roadrunner.examples;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

/**
 * Advanced autonomous example showing how to use RoadRunner actions
 * with FTC Dashboard for visualization.
 */
@Autonomous(name = "RoadRunner Advanced Auto", group = "Examples")
public class RoadRunnerAdvancedAutoExample extends NextFTCOpMode {
    private final Pose2d startPose = new Pose2d(0, 0, 0);

    private MecanumDrive drive;

    public RoadRunnerAdvancedAutoExample() {
        addComponents(
                new SubsystemComponent(MecanumDrive.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        drive = MecanumDrive.INSTANCE;
        drive.init(hardwareMap, imu, startPose);
    }

    @Override
    public void onStartButtonPressed() {
        // Run action-based trajectory using RoadRunner's Actions.runBlocking
        Actions.runBlocking(
                drive.actionBuilder(startPose)
                        // Drive forward
                        .lineToX(24)
                        // Spline to scoring position
                        .splineTo(new Vector2d(48, 24), Math.toRadians(90))
                        // Wait 1 second
                        .waitSeconds(1.0)
                        // Drive back
                        .lineToY(0)
                        .build()
        );
    }

    @Override
    public void onUpdate() {
        if (drive != null && drive.localizer != null) {
            telemetry.addData("x", drive.localizer.getPose().position.x);
            telemetry.addData("y", drive.localizer.getPose().position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
            telemetry.update();
        }
    }
}
