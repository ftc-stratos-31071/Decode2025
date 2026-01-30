package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.WaitCmd;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.PedroConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;

@Autonomous(name = "CloseBlueAuto")
public class CloseBlueAuto extends NextFTCOpMode {

    private Follower follower;
    private PathChain path1;
    private PathChain path2;
    private PathChain path3;
    private PathChain path4;
    private PathChain path5;

    public CloseBlueAuto() {
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
                .addPath(new BezierCurve(
                        new Pose(19.500, 123.500),
                        new Pose(89.000, 60.000),
                        new Pose(40.000, 59.500)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(142.5),
                        Math.toRadians(180)
                )
                .build();


        path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(40.000, 59.500),
                                new Pose(10.000, 59.500)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(10.000, 59.500),
                        new Pose(60.500, 82.500)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(210)
                )
                .build();

        path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(60.500, 82.500),
                                new Pose(33.000, 63.000),
                                new Pose(15.500, 67.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(180))

                .build();

        path5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.500, 64.500),
                                new Pose(13.000, 59.000),
                                new Pose(12.000, 59.500)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(150))
                .build();
    }

    @Override
    public void onInit() {
        follower = PedroConstants.createFollower(hardwareMap);
        follower.setStartingPose(
                new Pose(19.500, 123.500, Math.toRadians(142.5))
        );
        buildPaths();
    }

    @Override
    public void onStartButtonPressed() {
        new SequentialGroup(
                new FollowPath(path1),
                Intake.INSTANCE.moveIntake(IntakeConstants.intakePower),
                new FollowPath(path2),
                Intake.INSTANCE.moveIntake(0.0),
                new FollowPath(path3),
                Intake.INSTANCE.moveIntake(-IntakeConstants.intakePower),
                Intake.INSTANCE.moveTransfer(-IntakeConstants.intakePower),
                WaitCmd.create(0.5),
                Intake.INSTANCE.moveIntake(0.0),
                Intake.INSTANCE.moveTransfer(0.0),
                new FollowPath(path4),
                Intake.INSTANCE.moveIntake(IntakeConstants.intakePower),
                new FollowPath(path5),
                WaitCmd.create(2.0),
                Intake.INSTANCE.moveIntake(0.0)
        ).invoke();
    }
}