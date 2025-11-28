package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.FollowPath;
import org.firstinspires.ftc.teamcode.constants.PedroConstants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "Auto2 - Pedro + Turret")
public class Auto2 extends NextFTCOpMode {

    // Define starting and ending poses
    private final Pose startPose = new Pose(0.0, 0.0, Math.toRadians(0.0));
    private final Pose forwardPose = new Pose(5.0, 0.0, Math.toRadians(0.0)); // 5 inches forward

    private PathChain moveForward;
    private Follower follower;

    public Auto2() {
        addComponents(
                new PedroComponent(PedroConstants::createFollower),
                new SubsystemComponent(Turret.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }

    /**
     * Build the path that moves the robot forward 5 inches
     */
    private void buildPaths() {
        moveForward = follower.pathBuilder()
                .addPath(new BezierLine(startPose, forwardPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), forwardPose.getHeading())
                .build();
    }

    @Override
    public void onInit() {
        // Create the follower instance directly
        follower = PedroConstants.createFollower(hardwareMap);

        // Set the starting pose for the follower
        follower.setStartingPose(startPose);

        // Build the paths
        buildPaths();

        // Zero the turret motor on initialization
        Turret.INSTANCE.turret.zeroed();
    }

    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }

    /**
     * Autonomous routine: Move forward 5 inches, then sweep the turret
     */
    private Command autonomousRoutine() {
        return new SequentialGroup(
                // Move forward 5 inches using Pedro pathfollowing
                new FollowPath(follower, moveForward),

                // Small delay to ensure robot has settled
                new Delay(0.5),

                // Turret sweep sequence: left -> right -> center
                Turret.INSTANCE.runTurret(-45.0),  // Turn left 45 degrees
                new Delay(0.5),

                Turret.INSTANCE.runTurret(45.0),   // Turn right 45 degrees
                new Delay(0.5),

                Turret.INSTANCE.runTurret(0.0)     // Return to center
        );
    }
}
