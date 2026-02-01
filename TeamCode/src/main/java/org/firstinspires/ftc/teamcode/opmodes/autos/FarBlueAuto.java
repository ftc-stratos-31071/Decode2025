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

import org.firstinspires.ftc.teamcode.commands.AutoDriveTimeoutCmd;
import org.firstinspires.ftc.teamcode.commands.WaitCmd;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.PedroConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
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
    private PathChain path7;

    double servoPos = ShooterConstants.defaultPos;

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
        servoPos = ShooterConstants.defaultPos;
    }

    public void buildPaths() {

        path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.000, 8.000),

                                new Pose(3, 8.322)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(3, 8.322),
                                new Pose(56.000, 8)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();


        path3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(56.000, 8),
                                new Pose(48.548, 38.514),
                                new Pose(18.597, 35.774)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                                new Pose(18.597, 35.774),
                                new Pose(21.990, 33.279),
                                new Pose(25.383, 30.784),
                                new Pose(28.776, 28.289),
                                new Pose(32.169, 25.794),
                                new Pose(35.561, 23.299),
                                new Pose(38.954, 20.804),
                                new Pose(42.347, 18.309),
                                new Pose(45.740, 15.813),
                                new Pose(49.132, 13.318),
                                new Pose(56, 8)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56, 8),
                                new Pose(3, 8.192)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(3, 8.192),
                                new Pose(56, 8)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56, 8),
                                new Pose(36, 8)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    @Override
    public void onInit() {
        follower = PedroConstants.createFollower(hardwareMap);
        follower.setStartingPose(
                new Pose(56.000, 8.000, Math.toRadians(180))
        );
        buildPaths();
    }

    @Override
    public void onStartButtonPressed() {
        new SequentialGroup(
                //AutoDriveTimeoutCmd.create(new FollowPath(path1),1),
                WaitCmd.create(1.4),
                servoPos = servoPos + 0.1;
                Shooter.INSTANCE.moveServo(servoPos).schedule();
                Intake.INSTANCE.moveIntake(IntakeConstants.intakePower),
                Intake.INSTANCE.moveTransfer(IntakeConstants.intakePower),
                AutoDriveTimeoutCmd.create(new FollowPath(path1),2),
                new FollowPath(path2),
                Intake.INSTANCE.moveIntake(-IntakeConstants.intakePower),
                Intake.INSTANCE.moveTransfer(-IntakeConstants.intakePower),
                WaitCmd.create(1.4),
                Intake.INSTANCE.moveIntake(IntakeConstants.intakePower),
                Intake.INSTANCE.moveTransfer(IntakeConstants.intakePower),
                new FollowPath(path3),
                new FollowPath(path4),
                Intake.INSTANCE.moveIntake(-IntakeConstants.intakePower),
                Intake.INSTANCE.moveTransfer(-IntakeConstants.intakePower),
                WaitCmd.create(1.4),
                Intake.INSTANCE.moveIntake(IntakeConstants.intakePower),
                Intake.INSTANCE.moveTransfer(IntakeConstants.intakePower),
                AutoDriveTimeoutCmd.create(new FollowPath(path5),2),
                new FollowPath(path6),
                Intake.INSTANCE.moveIntake(-IntakeConstants.intakePower),
                Intake.INSTANCE.moveTransfer(-IntakeConstants.intakePower),
                new FollowPath(path7)
        ).invoke();
    }
}