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
import org.firstinspires.ftc.teamcode.constants.AutoPoseMemory;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.PedroConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret2;

@Autonomous(name = "FarRedTrackAuto")
public class FarRedTrackAuto extends NextFTCOpMode {

    private Follower follower;

    private PathChain path1;
    private PathChain path2;
    private PathChain path3;
    private PathChain path4;
    private PathChain path5;
    private PathChain path6;
    private PathChain path7;

    private double turretPos = 0.0;
    private double lastPedroX = 0.0;
    private double lastPedroY = 0.0;
    private double lastPedroHeadingDeg = 0.0;
    private double lastTraditionalHeadingDeg = 0.0;
    private double lastFtcX = 0.0;
    private double lastFtcY = 0.0;

    public FarRedTrackAuto() {
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
                                p(56.000, 8.000),

                                p(3, 10.322)
                        )
                ).setLinearHeadingInterpolation(h(180), h(180))
                .build();

        path2 = follower.pathBuilder().addPath(
                        new BezierLine(
                                p(3, 10.322),
                                p(56.000, 10)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();


        path3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                p(56.000, 10),
                                p(48.548, 38.514),
                                p(13.597, 35.774)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                                p(13.597, 35.774),
                                p(21.990, 33.279),
                                p(25.383, 30.784),
                                p(28.776, 28.289),
                                p(32.169, 25.794),
                                p(35.561, 23.299),
                                p(38.954, 20.804),
                                p(42.347, 18.309),
                                p(45.740, 15.813),
                                p(49.132, 13.318),
                                p(56, 10)
                        )
                ).setLinearHeadingInterpolation(h(180), h(180))
                .build();

        path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                p(56, 10),
                                p(3, 27)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                p(3, 27),
                                p(56, 10)
                        )
                ).setLinearHeadingInterpolation(h(180), h(180))
                .build();

        path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                p(56, 10),
                                p(36, 10)
                        )
                ).setLinearHeadingInterpolation(h(180), h(180))
                .build();
    }

    @Override
    public void onInit() {
        AutoPoseMemory.clear();

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
                ph(56.000, 8.000, 180)
        );
        buildPaths();
    }

    @Override
    public void onStartButtonPressed() {
        Shooter.INSTANCE.runRPM(ShooterConstants.farTargetRPM).schedule();
        new SequentialGroup(
                Turret2.INSTANCE.goToAngle(-65.0),
                Intake.INSTANCE.defaultPos(),
                WaitCmd.create(0.5),
                RapidFireTimeoutCmdFar.create(2000),
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
                RapidFireTimeoutCmdFar.create(2000), // 1000ms timeout
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
                RapidFireTimeoutCmdFar.create(2000),  // 1000ms timeout
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
                RapidFireTimeoutCmdFar.create(2000),  // 1000ms timeout
//
//                // Cycle 4: Drive and collect
                Intake.INSTANCE.moveServoPos(),  // CLOSE door (0.0)
                IntakeSeqCmd.create(),
                AutoDriveTimeoutCmd.create(new FollowPath(path1),2),
                new FollowPath(path2),
                Intake.INSTANCE.zeroPowerIntake(),
                Intake.INSTANCE.zeroPowerTransfer(),
//
//                // SHOOT
                Intake.INSTANCE.defaultPos(),
                WaitCmd.create(0.25),// OPEN door (0.7)
                RapidFireTimeoutCmdFar.create(2000),  // 1000ms timeout
//
//                // Cycle 5: Drive and collect
                Intake.INSTANCE.moveServoPos(),  // CLOSE door (0.0)
                Intake.INSTANCE.zeroPowerIntake(),
                Intake.INSTANCE.zeroPowerTransfer(),
//
//                // Final positioning (door can stay open or close - doesn't matter)
                new FollowPath(path7)
        ).invoke();
    }

    @Override
    public void onUpdate() {
        updatePose();

        double rightRPM = Shooter.INSTANCE.ticksPerSecondToRPM(Math.abs(Shooter.INSTANCE.rightMotor.getVelocity()));
        double leftRPM = Shooter.INSTANCE.ticksPerSecondToRPM(Math.abs(Shooter.INSTANCE.leftMotor.getVelocity()));
        double currentRPM = (rightRPM + leftRPM) / 2.0;



    }

    private void updatePose() {
        Pose pose = follower.getPose();
        lastPedroX = pose.getX();
        lastPedroY = pose.getY();
        lastPedroHeadingDeg = AutoPoseMemory.normalizeAngle(Math.toDegrees(pose.getHeading()));
        lastTraditionalHeadingDeg = AutoPoseMemory.pedroToTraditionalHeading(lastPedroHeadingDeg);
        lastFtcX = AutoPoseMemory.pedroToTraditionalX(lastPedroX, lastPedroY);
        lastFtcY = AutoPoseMemory.pedroToTraditionalY(lastPedroX, lastPedroY);
    }

    private Pose p(double x, double y) {
        return new Pose(
                AutoPoseMemory.blueToRedPedroX(x),
                AutoPoseMemory.blueToRedPedroY(y)
        );
    }

    private Pose ph(double x, double y, double headingDeg) {
        return new Pose(
                AutoPoseMemory.blueToRedPedroX(x),
                AutoPoseMemory.blueToRedPedroY(y),
                h(headingDeg)
        );
    }

    private double h(double headingDeg) {
        return Math.toRadians(AutoPoseMemory.blueToRedPedroHeading(headingDeg));
    }

    private double redTurretAngle(double blueTurretAngle) {
        return -blueTurretAngle;
    }

    @Override
    public void onStop() {
        updatePose();
        AutoPoseMemory.setFtcPose(lastFtcX, lastFtcY, lastTraditionalHeadingDeg);
        Turret2.INSTANCE.setAngle(0.0);
    }
}
