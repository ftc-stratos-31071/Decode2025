package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.IntakeSeqCmd;
import org.firstinspires.ftc.teamcode.commands.RapidFireTimeoutCmd;
import org.firstinspires.ftc.teamcode.commands.WaitCmd;
import org.firstinspires.ftc.teamcode.constants.AutoPoseMemory;
import org.firstinspires.ftc.teamcode.constants.PedroConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret2;

import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "CloseRedTrackAuto")
public class CloseRedTrackAuto extends NextFTCOpMode {

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

    private double lastPedroX = 0.0;
    private double lastPedroY = 0.0;
    private double lastPedroHeadingDeg = 0.0;
    private double lastTraditionalHeadingDeg = 0.0;
    private double lastFtcX = 0.0;
    private double lastFtcY = 0.0;

    public CloseRedTrackAuto() {
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
                                p(19.500, 123.500),
                                p(55.000, 84.500)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                p(55.000, 84.500),
                                p(56.500, 63.000),
                                p(45.000, 61.500)
                        )
                ).setLinearHeadingInterpolation(h(142.5), h(180))
                .build();

        path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                p(45.000, 61.500),
                                p(10.000, 61.500)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        p(10.000, 61.500),
                        p(55.000, 84.500)
                ))
                .setLinearHeadingInterpolation(
                        h(180),
                        h(210)
                )
                .build();

        path5 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                p(55.000, 84.500),
                                p(33.000, 63.000),
                                p(17.000, 67.000)
                        )
                ).setLinearHeadingInterpolation(h(210), h(180))
                .build();

        path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                p(17.000, 67.000),
                                p(13.000, 59.000),
                                p(9.500, 57.000)
                        )
                ).setLinearHeadingInterpolation(h(180), h(145))
                .build();

        path7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        p(9.500, 57.000),
                        p(55.000, 84.500)
                ))
                .setLinearHeadingInterpolation(
                        h(145),
                        h(210)
                )
                .build();

        path8 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                p(55.000, 84.500),
                                p(56.500, 87.000),
                                p(40.000, 85.500)
                        )
                ).setLinearHeadingInterpolation(h(210.0), h(180))
                .build();

        path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                p(40.000, 85.500),
                                p(15.000, 85.500)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        path10 = follower.pathBuilder()
                .addPath(new BezierLine(
                        p(15.000, 85.500),
                        p(55.000, 109.000)
                ))
                .setLinearHeadingInterpolation(
                        h(180),
                        h(210)
                )
                .build();
    }

    @Override
    public void onInit() {
        AutoPoseMemory.clear();

        Intake.INSTANCE.moveServoPos().schedule();
        Shooter.INSTANCE.setTargetRPM(0.0);
        Shooter.INSTANCE.setHood(ShooterConstants.servoPos).schedule();
        Turret2.INSTANCE.setAngle(0.0);
        Turret2.INSTANCE.goToAngle(0.0).schedule();

        follower = PedroConstants.createFollower(hardwareMap);
        follower.setStartingPose(
                ph(19.500, 123.500, 142.5)
        );
        buildPaths();
    }

    @Override
    public void onStartButtonPressed() {
        Shooter.INSTANCE.setTargetRPM(3700);
        Shooter.INSTANCE.runRPM(3700).schedule();
        Shooter.INSTANCE.setHood(ShooterConstants.servoPos).schedule();

        new SequentialGroup(
                IntakeSeqCmd.create(),
                new FollowPath(path1),
                Intake.INSTANCE.moveIntake(0.0),
                Intake.INSTANCE.moveTransfer(0.0),
                Intake.INSTANCE.defaultPos(),
                WaitCmd.create(0.2),
                RapidFireTimeoutCmd.create(1200),
                Intake.INSTANCE.moveServoPos(),
                new FollowPath(path2),
                IntakeSeqCmd.create(),
                new FollowPath(path3),
                Intake.INSTANCE.moveIntake(0.0),
                Intake.INSTANCE.moveTransfer(0.0),
                Turret2.INSTANCE.goToAngle(redTurretAngle(90.0)),
                new FollowPath(path4),
                Intake.INSTANCE.defaultPos(),
                WaitCmd.create(0.2),
                RapidFireTimeoutCmd.create(1200),
                Intake.INSTANCE.moveServoPos(),
                IntakeSeqCmd.create(),
                new FollowPath(path5),
                new FollowPath(path6),
                WaitCmd.create(0.75),
                Intake.INSTANCE.moveIntake(0.0),
                Intake.INSTANCE.moveTransfer(0.0),
                new FollowPath(path7),
                Intake.INSTANCE.defaultPos(),
                WaitCmd.create(0.2),
                RapidFireTimeoutCmd.create(1200),
                Intake.INSTANCE.moveServoPos(),
                IntakeSeqCmd.create(),
                new FollowPath(path5),
                new FollowPath(path6),
                WaitCmd.create(0.75),
                Intake.INSTANCE.moveIntake(0.0),
                Intake.INSTANCE.moveTransfer(0.0),
                new FollowPath(path7),
                Intake.INSTANCE.defaultPos(),
                WaitCmd.create(0.2),
                RapidFireTimeoutCmd.create(1500),
                Intake.INSTANCE.moveServoPos(),
                new FollowPath(path8),
                IntakeSeqCmd.create(),
                new FollowPath(path9),
                Intake.INSTANCE.moveIntake(0.0),
                Intake.INSTANCE.moveTransfer(0.0),
                Turret2.INSTANCE.goToAngle(redTurretAngle(70.0)),
                Shooter.INSTANCE.runRPMAuto(3500),
                Shooter.INSTANCE.setHood(ShooterConstants.servoPos + 0.3),
                new FollowPath(path10),
                Intake.INSTANCE.defaultPos(),
                WaitCmd.create(0.2),
                RapidFireTimeoutCmd.create(1500),
                Intake.INSTANCE.moveServoPos()
        ).invoke();
    }

    @Override
    public void onUpdate() {
        updatePose();



    }

    private void updatePose() {
        Pose pose = follower.getPose();
        lastPedroX = pose.getX();
        lastPedroY = pose.getY();
        lastPedroHeadingDeg = normalizeAngle(Math.toDegrees(pose.getHeading()));
        lastTraditionalHeadingDeg = AutoPoseMemory.pedroToTraditionalHeading(lastPedroHeadingDeg);

        lastFtcX = AutoPoseMemory.pedroToTraditionalX(lastPedroX, lastPedroY);
        lastFtcY = AutoPoseMemory.pedroToTraditionalY(lastPedroX, lastPedroY);
    }

    private double normalizeAngle(double degrees) {
        degrees = degrees % 360.0;
        if (degrees < 0) degrees += 360.0;
        return degrees;
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
