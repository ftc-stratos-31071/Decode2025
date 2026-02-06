package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.teamcode.constants.PedroConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret2;

@Autonomous(name = "BlueTrackingAuto")
public class BlueTrackingAuto extends NextFTCOpMode {

    private Follower follower;
    private PathChain turnPath;

    // Match FarBlueAuto start pose
    public static double START_X = 56.0;
    public static double START_Y = 8.0;
    public static double START_HEADING = 180.0;

    public static double TURN_DEGREES = 90.0;

    public BlueTrackingAuto() {
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
        double startHeadingRad = Math.toRadians(START_HEADING);
        double endHeadingRad = Math.toRadians(START_HEADING + TURN_DEGREES);

        // Small translation to allow the follower to execute a heading change.
        double endX = START_X + 0.1;
        double endY = START_Y;

        turnPath = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(START_X, START_Y),
                                new Pose(endX, endY)
                        )
                ).setLinearHeadingInterpolation(startHeadingRad, endHeadingRad)
                .build();
    }

    @Override
    public void onInit() {
        follower = PedroConstants.createFollower(hardwareMap);
        follower.setStartingPose(
                new Pose(START_X, START_Y, Math.toRadians(START_HEADING))
        );
        buildPaths();
    }

    @Override
    public void onStartButtonPressed() {
        new FollowPath(turnPath).schedule();
    }
}
