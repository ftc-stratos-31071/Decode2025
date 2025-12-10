package org.firstinspires.ftc.teamcode.roadrunner.examples;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import dev.nextftc.ftc.NextFTCOpMode;

/**
 * Example TeleOp OpMode using NextFTC with RoadRunner.
 * This demonstrates field-centric driving with the mecanum drive.
 */
@TeleOp(name = "RoadRunner TeleOp Example", group = "Examples")
public class RoadRunnerTeleOpExample extends NextFTCOpMode {
    private MecanumDrive drive;

    @Override
    public void onInit() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
    }

    @Override
    public void onUpdate() {
        // Update pose estimate
        drive.updatePoseEstimate();

        // Get joystick inputs
        double forward = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        // Apply field-centric transformation
        double heading = drive.localizer.getPose().heading.toDouble();
        double rotatedForward = forward * Math.cos(-heading) - strafe * Math.sin(-heading);
        double rotatedStrafe = forward * Math.sin(-heading) + strafe * Math.cos(-heading);

        // Set drive powers
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(rotatedForward, rotatedStrafe),
                turn
        ));

        // Display telemetry
        telemetry.addData("x", drive.localizer.getPose().position.x);
        telemetry.addData("y", drive.localizer.getPose().position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.localizer.getPose().heading.toDouble()));
        telemetry.update();
    }
}
