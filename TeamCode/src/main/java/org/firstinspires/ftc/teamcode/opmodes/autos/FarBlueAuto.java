package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.subsystems.Shooter.*;

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
import org.firstinspires.ftc.teamcode.commands.IntakeSeqCmd;
import org.firstinspires.ftc.teamcode.commands.RapidFireCmd;
import org.firstinspires.ftc.teamcode.commands.RapidFireTimeoutCmdFar;
import org.firstinspires.ftc.teamcode.commands.ShootBallSteadyAutoCmd;
import org.firstinspires.ftc.teamcode.commands.ShootBallSteadyCmd;
import org.firstinspires.ftc.teamcode.commands.WaitCmd;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.PedroConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret2;

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

    private double turretPos = 0.0;

    public FarBlueAuto() {
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
                                new Pose(3, 27)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(3, 27),
                                new Pose(56, 8)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56, 8),
                                new Pose(36, 8)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }

    @Override
    public void onInit() {
        // CRITICAL: Reset all subsystem state from previous runs
        Shooter.INSTANCE.runRPM(0.0).schedule();
        Intake.INSTANCE.setIntakePower(0.0);  // Stop intake motors
        Intake.INSTANCE.setTransferPower(0.0);  // Stop transfer motors

        // Reset hood to lower angle for far shots
        Shooter.INSTANCE.setHood(ShooterConstants.farHoodPos).schedule();  // 0.4

        // Close door on init
        Intake.INSTANCE.moveServoPos().schedule();

        // Reset turret angle
        Turret2.INSTANCE.setAngle(0.0);
        Turret2.INSTANCE.goToAngle(0.0);
        Shooter.INSTANCE.stop();

        follower = PedroConstants.createFollower(hardwareMap);
        follower.setStartingPose(
                new Pose(56.000, 8.000, Math.toRadians(180))
        );
        buildPaths();
    }

    @Override
    public void onStartButtonPressed() {
        Shooter.INSTANCE.runRPM(4650).schedule();
        new SequentialGroup(
                Turret2.INSTANCE.goToAngle(70.0),
                Intake.INSTANCE.defaultPos(),
                WaitCmd.create(0.25),
                RapidFireTimeoutCmdFar.create(3000),
                Intake.INSTANCE.moveServoPos(),
                IntakeSeqCmd.create(),
                AutoDriveTimeoutCmd.create(new FollowPath(path1),2),
                new FollowPath(path2),
                Intake.INSTANCE.zeroPowerIntake(),
                Intake.INSTANCE.zeroPowerTransfer(),
////
                // SHOOT collected balls
                Intake.INSTANCE.defaultPos(),
                WaitCmd.create(0.25),// OPEN door (0.7)
                RapidFireTimeoutCmdFar.create(3000), // 1000ms timeout
//
//                // Cycle 2: Drive and collect
                Intake.INSTANCE.moveServoPos(),  // CLOSE door (0.0)
                IntakeSeqCmd.create(),
                new FollowPath(path3),
                new FollowPath(path4),
                Intake.INSTANCE.zeroPowerIntake(),
                Intake.INSTANCE.zeroPowerTransfer(),
//
//                // SHOOT
                Intake.INSTANCE.defaultPos(),
                WaitCmd.create(0.25),// OPEN door (0.7)
                RapidFireTimeoutCmdFar.create(3000),  // 1000ms timeout
//
//                // Cycle 3: Drive and collect
                Intake.INSTANCE.moveServoPos(),  // CLOSE door (0.0)
                IntakeSeqCmd.create(),
                AutoDriveTimeoutCmd.create(new FollowPath(path5),2),
                new FollowPath(path6),
                Intake.INSTANCE.zeroPowerIntake(),
                Intake.INSTANCE.zeroPowerTransfer(),
//
//                // SHOOT
                Intake.INSTANCE.defaultPos(),
                WaitCmd.create(0.25),// OPEN door (0.7)
                RapidFireTimeoutCmdFar.create(3000)  // 1000ms timeout
//
//                // Cycle 4: Drive and collect
//                Intake.INSTANCE.moveServoPos(),  // CLOSE door (0.0)
//                IntakeSeqCmd.create(),
//                AutoDriveTimeoutCmd.create(new FollowPath(path1),2),
//                new FollowPath(path2),
//                Intake.INSTANCE.zeroPowerIntake(),
//                Intake.INSTANCE.zeroPowerTransfer(),
//
//                // SHOOT
//                Intake.INSTANCE.defaultPos(),  // OPEN door (0.7)
//                RapidFireTimeoutCmd.create(3000),  // 1000ms timeout
//
//                // Cycle 5: Drive and collect
//                Intake.INSTANCE.moveServoPos(),  // CLOSE door (0.0)
//                IntakeSeqCmd.create(),
//                AutoDriveTimeoutCmd.create(new FollowPath(path1),2),
//                new FollowPath(path2),
//                Intake.INSTANCE.zeroPowerIntake(),
//                Intake.INSTANCE.zeroPowerTransfer(),
//
//                // FINAL SHOOT
//                Intake.INSTANCE.defaultPos(),  // OPEN door (0.7)
//                RapidFireTimeoutCmd.create(3000),  // 1000ms timeout
//
//                // Final positioning (door can stay open or close - doesn't matter)
//                new FollowPath(path7)
        ).invoke();
    }

    @Override
    public void onUpdate() {
        double rightRPM = Shooter.INSTANCE.ticksPerSecondToRPM(Math.abs(Shooter.INSTANCE.rightMotor.getVelocity()));
        double leftRPM = Shooter.INSTANCE.ticksPerSecondToRPM(Math.abs(Shooter.INSTANCE.leftMotor.getVelocity()));
        double currentRPM = (rightRPM + leftRPM) / 2.0;
        telemetry.addData("Current RPM", currentRPM);
        telemetry.update();
    }
}

