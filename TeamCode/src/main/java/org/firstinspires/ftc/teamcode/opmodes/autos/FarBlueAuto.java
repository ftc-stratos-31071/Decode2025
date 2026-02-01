package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.hardware.ams.AMSColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;

import org.firstinspires.ftc.teamcode.commands.WaitCmd;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.PedroConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Autonomous(name = "FarBlueAuto")
public class FarBlueAuto extends NextFTCOpMode {

    private Follower follower;

    private PathChain path1;
    private PathChain path2;
    private PathChain path3;
    private PathChain path4;
    private PathChain path5;
    private PathChain path6;

    public FarBlueAuto() {
        addComponents(
                new SubsystemComponent(
                        Intake.INSTANCE,
                        Shooter.INSTANCE
                ),
                new PedroComponent(PedroConstants::createFollower),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    public void buildPaths() {

        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56.000, 8.000),
                        new Pose(56.000, 10.000)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(90),
                        Math.toRadians(105)
                )
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(56.000, 10.000),
                        new Pose(24.357, 19.071),
                        new Pose(10.286, 23.500)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(105),
                        Math.toRadians(245)
                )
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(10.286, 23.500),
                        new Pose(6.500, 5.357),
                        new Pose(6.99, 9.9)
                ))
                .setConstantHeadingInterpolation(
                        Math.toRadians(-95)
                )
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(6.99, 9.9),
                        new Pose(24.429, 16.286)
                ))
                .setConstantHeadingInterpolation(
                        Math.toRadians(-95)
                )
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(24.429, 16.286),
                        new Pose(56.000, 10.000)
                ))
                .setConstantHeadingInterpolation(
                        Math.toRadians(105)
                )
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(56.000, 10.000),
                        new Pose(36.000, 10.000)
                ))
                .setConstantHeadingInterpolation(
                        Math.toRadians(105)
                )
                .build();
    }

    @Override
    public void onInit() {
        follower = PedroConstants.createFollower(hardwareMap);
        follower.setStartingPose(
                new Pose(56.000, 8.000, Math.toRadians(90))
        );
        buildPaths();
    }

    @Override
    public void onStartButtonPressed() {
        new SequentialGroup(
                //AutoDriveTimeoutCmd.create(new FollowPath(path3), 1.0),
                new FollowPath(path1),
                WaitCmd.create(1.4),
                new FollowPath(path2),
                Intake.INSTANCE.moveIntake(IntakeConstants.intakePower),
                Intake.INSTANCE.moveTransfer(IntakeConstants.intakePower),
                WaitCmd.create(2.0),
                Intake.INSTANCE.moveIntake(0.0),
                Intake.INSTANCE.moveTransfer(0.0),
                new FollowPath(path3),
                new FollowPath(path4),
                new FollowPath(path5),
                WaitCmd.create(1.4),
                new FollowPath(path6)
        ).invoke();
    }
}