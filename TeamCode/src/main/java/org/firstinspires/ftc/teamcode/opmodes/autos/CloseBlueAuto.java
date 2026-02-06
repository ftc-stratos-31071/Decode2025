package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.IntakeSeqCmd;
import org.firstinspires.ftc.teamcode.commands.RapidFireCmd;
import org.firstinspires.ftc.teamcode.commands.RapidFireTimeoutCmd;
import org.firstinspires.ftc.teamcode.commands.WaitCmd;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.PedroConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Turret2;

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
    private PathChain path6;
    private PathChain path7;
    private PathChain path8;
    private PathChain path9;
    private PathChain path10;

    public CloseBlueAuto() {
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

    public void buildPaths() {

        path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(19.500, 123.500),

                                new Pose(55.000, 84.500)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(55.000, 84.500),
                                new Pose(56.500, 63.000),
                                new Pose(40.000, 61.500)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(142.5), Math.toRadians(180))

                .build();


        path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(40.000, 61.500),
                                new Pose(10.000, 61.500)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(10.000, 61.500),
                        new Pose(55.000, 84.500)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(210)
                )
                .build();

        path5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(55.000, 84.500),
                                new Pose(33.000, 63.000),
                                new Pose(15.500, 67.000)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(210), Math.toRadians(180))

                .build();

        path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.500, 67.000),
                                new Pose(13.000, 59.000),
                                new Pose(8.000, 57.500)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(8.000, 57.500),
                        new Pose(55.000, 84.500)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(110),
                        Math.toRadians(210)
                )
                .build();

        path8 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(55.000, 84.500),
                                new Pose(56.500, 87.000),
                                new Pose(40.000, 85.500)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(210.0), Math.toRadians(180))
                .build();

        path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(40.000, 85.500),
                                new Pose(20.000, 85.500)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        path10 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(10.000, 85.500),
                        new Pose(55.000, 109.000)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(210)
                )
                .build();
    }

    @Override
    public void onInit() {
        Intake.INSTANCE.moveServoPos().schedule();
        Shooter.INSTANCE.setTargetRPM(0.0);
        Turret2.INSTANCE.setAngle(0.0);
        Turret2.INSTANCE.goToAngle(0.0).schedule();
        Shooter.INSTANCE.setHood(ShooterConstants.servoPos).schedule();
        follower = PedroConstants.createFollower(hardwareMap);
        follower.setStartingPose(
                new Pose(19.500, 123.500, Math.toRadians(142.5))
        );
        buildPaths();
    }

    @Override
    public void onStartButtonPressed() {
        Shooter.INSTANCE.setTargetRPM(3700);
        Shooter.INSTANCE.runRPM(3700).schedule();
        Shooter.INSTANCE.setHood(ShooterConstants.servoPos).schedule();
        Turret2.INSTANCE.goToAngle(10.0).schedule();
        new SequentialGroup(
                IntakeSeqCmd.create(),
                new FollowPath(path1),
                Intake.INSTANCE.moveIntake(0.0),
                Intake.INSTANCE.moveTransfer(0.0),
                Intake.INSTANCE.defaultPos(),
                WaitCmd.create(0.25),
                RapidFireTimeoutCmd.create(900),
                Intake.INSTANCE.moveServoPos(),
                new FollowPath(path2),
                IntakeSeqCmd.create(),
                new FollowPath(path3),
                Intake.INSTANCE.moveIntake(0.0),
                Intake.INSTANCE.moveTransfer(0.0),
                Turret2.INSTANCE.goToAngle(110.0),
                new FollowPath(path4),
                Intake.INSTANCE.defaultPos(),
                WaitCmd.create(0.25),
                RapidFireTimeoutCmd.create(900),
                Intake.INSTANCE.moveServoPos(),
                IntakeSeqCmd.create(),
                new FollowPath(path5),
                new FollowPath(path6),
                WaitCmd.create(0.5),
                new FollowPath(path7),
                Intake.INSTANCE.moveIntake(0.0),
                Intake.INSTANCE.moveTransfer(0.0),
                Intake.INSTANCE.defaultPos(),
                WaitCmd.create(0.25),
                RapidFireTimeoutCmd.create(900),
                Intake.INSTANCE.moveServoPos(),
                IntakeSeqCmd.create(),
                new FollowPath(path5),
                new FollowPath(path6),
                WaitCmd.create(0.5),
                new FollowPath(path7),
                Intake.INSTANCE.moveIntake(0.0),
                Intake.INSTANCE.moveTransfer(0.0),
                Intake.INSTANCE.defaultPos(),
                WaitCmd.create(0.25),
                RapidFireTimeoutCmd.create(900),
                Intake.INSTANCE.moveServoPos(),
                IntakeSeqCmd.create(),
                new FollowPath(path5),
                WaitCmd.create(0.5),
                new FollowPath(path6),
                WaitCmd.create(0.5),
                new FollowPath(path7),
                Intake.INSTANCE.moveIntake(0.0),
                Intake.INSTANCE.moveTransfer(0.0),
                Intake.INSTANCE.defaultPos(),
                WaitCmd.create(0.25),
                RapidFireTimeoutCmd.create(900),
                Intake.INSTANCE.moveServoPos(),
                new FollowPath(path8),
                IntakeSeqCmd.create(),
                new FollowPath(path9),
                Intake.INSTANCE.moveIntake(0.0),
                Intake.INSTANCE.moveTransfer(0.0),
                Turret2.INSTANCE.goToAngle(80.0),
                Shooter.INSTANCE.setHood(ShooterConstants.servoPos + 0.4),
                new FollowPath(path10),
                Intake.INSTANCE.defaultPos(),
                WaitCmd.create(0.25),
                RapidFireTimeoutCmd.create(900),
                Intake.INSTANCE.moveServoPos()
        ).invoke();
    }
}