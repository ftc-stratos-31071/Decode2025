package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;

import dev.nextftc.core.commands.Command;

/**
 * NextFTC Command for following a RoadRunner trajectory.
 */
public class FollowTrajectory extends Command {
    private final MecanumDrive drive;
    private final TimeTrajectory trajectory;
    private double beginTs = -1;

    public FollowTrajectory(MecanumDrive drive, TimeTrajectory trajectory) {
        this.drive = drive;
        this.trajectory = trajectory;
    }

    @Override
    public void start() {
        beginTs = System.nanoTime() * 1e-9;
    }

    @Override
    public void update() {
        double t = System.nanoTime() * 1e-9 - beginTs;

        Pose2dDual<Time> txWorldTarget = trajectory.get(t);

        PoseVelocity2d robotVelRobot = drive.updatePoseEstimate();

        PoseVelocity2dDual<Time> command = drive.controller
                .compute(txWorldTarget, drive.localizer.getPose(), robotVelRobot);

        drive.setDrivePowersFF(command);
    }

    @Override
    public boolean isDone() {
        if (beginTs < 0) return false;
        double t = System.nanoTime() * 1e-9 - beginTs;
        return t >= trajectory.duration;
    }

    @Override
    public void stop(boolean interrupted) {
        // Stop all motors
        drive.frontLeft.setPower(0);
        drive.backLeft.setPower(0);
        drive.backRight.setPower(0);
        drive.frontRight.setPower(0);
    }
}
