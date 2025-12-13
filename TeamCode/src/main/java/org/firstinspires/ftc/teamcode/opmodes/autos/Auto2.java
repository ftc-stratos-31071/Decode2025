package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

/**
 * Autonomous OpMode using NextFTC with RoadRunner.
 *
 * This auto follows the paths from MeepMeepTesting:
 * 1. Score preload (drive backwards to scoring position)
 * 2. Pick up artifacts line 1 (spline to pickup zone)
 * 3. Return to scoring position
 * 4. Pick up line 2
 * 5. Return to scoring position
 */
@Autonomous(name = "Auto2")
public class Auto2 extends NextFTCOpMode {

    // Start pose matching MeepMeep configuration
    // Position: (-52.5, -51.5) with heading -124 degrees
    private final Pose2d startPose = new Pose2d(-52.5, -51.5, Math.toRadians(-124));

    private MecanumDrive drive;
    private Action trajectoryAction;
    private boolean actionRunning = false;

    public Auto2() {
        addComponents(
                new SubsystemComponent(Turret.INSTANCE, MecanumDrive.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        // Get IMU from hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");

        // Initialize the MecanumDrive singleton with hardware and start pose
        drive = MecanumDrive.INSTANCE;
        drive.init(hardwareMap, imu, startPose);

        // Build the complete trajectory action matching MeepMeep paths
        trajectoryAction = drive.actionBuilder(startPose)

                // --- score preload ---
                .setReversed(true)
                .strafeTo(new Vector2d(-14.0, -13.0))

                // --- pick up artifacts line 1 ---
                .setReversed(false)
                .splineTo(
                        new Vector2d(-12.0, -54.0),
                        Math.toRadians(270)
                )

                // --- back to shoot position ---
                .setReversed(true)
                .splineToLinearHeading(
                        new Pose2d(-14.0, -13.0, Math.toRadians(225)),
                        Math.toRadians(90)
                )

                // --- pick up line 2 ---
                .setReversed(false)
                .splineTo(
                        new Vector2d(12, -48.0),
                        Math.toRadians(270)
                )

                // --- return to score ---
                .setReversed(true)
                .splineToLinearHeading(
                        new Pose2d(-14.0, -13.0, Math.toRadians(225)),
                        Math.toRadians(90)
                )

                .build();

        // Zero the turret motor on initialization
        Turret.INSTANCE.turret.zeroed();
        Turret.INSTANCE.disableManualControl();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Start Pose", "%.1f, %.1f, %.1f°",
                startPose.position.x, startPose.position.y, Math.toDegrees(startPose.heading.toDouble()));
        telemetry.update();
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
        if (drive != null && drive.localizer != null) {
            Pose2d currentPose = drive.localizer.getPose();
            telemetry.addData("x", currentPose.position.x);
            telemetry.addData("y", currentPose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(currentPose.heading.toDouble()));
            telemetry.addData("Action Running", actionRunning);
            telemetry.update();
        }
    }
}
