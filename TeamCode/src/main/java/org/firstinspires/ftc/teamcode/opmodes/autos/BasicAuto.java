package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.constants.PedroConstants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;


@Autonomous(name = "Basic Auto", group = "Competition")
public class BasicAuto extends NextFTCOpMode {

    // Starting pose - matches Path1 starting position
    private final Pose startPose = new Pose(19.500, 123.500, Math.toRadians(0.0));

    private Follower follower;
    private Paths paths;

    public BasicAuto() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Turret.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }

    /**
     * Container class for all autonomous paths
     */
    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(19.500, 123.500), new Pose(58.000, 85.000))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(58.000, 85.000),
                                    new Pose(61.000, 59.500),
                                    new Pose(33.000, 60.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(33.000, 60.000), new Pose(58.000, 85.500))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(58.000, 85.500),
                                    new Pose(46.000, 33.000),
                                    new Pose(13.000, 59.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(13.000, 59.000),
                                    new Pose(31.500, 53.000),
                                    new Pose(29.000, 73.500),
                                    new Pose(58.000, 85.000)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(58.000, 85.000), new Pose(33.000, 84.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(33.000, 84.000), new Pose(58.500, 85.000))
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
        }
    }

    /**
     * Command to follow a Pedro PathChain
     */
    private class FollowPathCommand extends Command {
        private final Follower follower;
        private final PathChain path;

        public FollowPathCommand(Follower follower, PathChain path) {
            this.follower = follower;
            this.path = path;
        }

        @Override
        public void start() {
            follower.followPath(path);
        }

        @Override
        public void update() {
            follower.update();
        }

        @Override
        public boolean isDone() {
            return !follower.isBusy();
        }
    }

    @Override
    public void onInit() {
        // Create follower
        follower = PedroConstants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        // Build all paths
        paths = new Paths(follower);

        // Initialize subsystems same as Teleop
        Intake.INSTANCE.defaultPos.schedule();
        Shooter.INSTANCE.defaultPos.schedule();
        Turret.INSTANCE.turret.zeroed();

        telemetry.addData("Status", "✓ Initialized");
        telemetry.addData("Start Pose", "(%.1f, %.1f) @ %.0f°",
                startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()));
        telemetry.addData("Paths Loaded", "7 paths ready");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }

    /**
     * Autonomous routine - movement only, no scoring
     */
    private Command autonomousRoutine() {
        return new SequentialGroup(
                // Path 1: Drive to scoring position
                new FollowPathCommand(follower, paths.Path1),
                
                // Path 2: Drive to first pickup
                new FollowPathCommand(follower, paths.Path2),
                
                // Path 3: Return to scoring
                new FollowPathCommand(follower, paths.Path3),
                
                // Path 4: Drive to second pickup
                new FollowPathCommand(follower, paths.Path4),
                
                // Path 5: Return to scoring
                new FollowPathCommand(follower, paths.Path5),
                
                // Path 6: Align
                new FollowPathCommand(follower, paths.Path6),

                // Path 7: Park
                new FollowPathCommand(follower, paths.Path7)
        );
    }


    @Override
    public void onUpdate() {
        // Display telemetry
        if (follower != null) {
            Pose currentPose = follower.getPose();

            telemetry.addData("X Position", "%.2f", currentPose.getX());
            telemetry.addData("Y Position", "%.2f", currentPose.getY());
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(currentPose.getHeading()));
            telemetry.addData("Path Following", follower.isBusy() ? "ACTIVE" : "IDLE");
            telemetry.update();
        }
    }

    @Override
    public void onStop() {
        if (follower != null) {
            follower.breakFollowing();
        }
    }
}
