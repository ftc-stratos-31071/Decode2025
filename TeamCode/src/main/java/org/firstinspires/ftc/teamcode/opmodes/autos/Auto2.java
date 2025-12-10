package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

/**
 * Autonomous OpMode demonstrating RoadRunner integration with NextFTC.
 *
 * This example:
 * 1. Follows a complex 7-path sequence using RoadRunner trajectory following
 * 2. Performs a turret sweep sequence (left -> right -> center)
 *
 * The MecanumDrive is configured with Pinpoint localizer.
 */
@Autonomous(name = "Auto2")
public class Auto2 extends NextFTCOpMode {

    // Start pose matching MeepMeep configuration
    // Position: (-52.5, 51.5) with heading -45 degrees
    private final Pose2d startPose = new Pose2d(-52.5, 51.5, Math.toRadians(-45.0));

    private MecanumDrive drive;
    private Action trajectoryAction;
    private boolean actionRunning = false;

    public Auto2() {
        addComponents(
                new SubsystemComponent(Turret.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        // Create the MecanumDrive instance with Pinpoint localizer
        drive = new MecanumDrive(hardwareMap, startPose);

        // Build the complete trajectory action matching MeepMeep paths
        trajectoryAction = drive.actionBuilder(startPose)
                // --- Path1: BezierLine, reversed ---
                // Drive backwards from start to (-14.0, 13.0)
                .setReversed(true)
                .strafeTo(new Vector2d(-14.0, 13.0))

                // --- Path2: BezierCurve, tangent heading ---
                // Spline to (-39.0, -12.0) with end tangent ~178.98°
                .setReversed(false)
                .splineTo(
                        new Vector2d(-39.0, -12.0),
                        3.124  // radians (~178.98°)
                )

                // --- Path3: BezierLine, reversed ---
                // Drive backwards to (-14.0, 13.5)
                .setReversed(true)
                .strafeTo(new Vector2d(-14.0, 13.5))

                // --- Path4: BezierCurve, tangent heading ---
                // Spline to (-59.0, -13.0) with end tangent ~141.77°
                .setReversed(false)
                .splineTo(
                        new Vector2d(-59.0, -13.0),
                        2.474  // radians (~141.77°)
                )

                // --- Path5: BezierCurve, reversed, tangent heading ---
                // Spline backwards to (-14.0, 13.0) with end tangent ~21.63°
                .setReversed(true)
                .splineTo(
                        new Vector2d(-14.0, 13.0),
                        0.378  // radians (~21.63°)
                )

                // --- Path6: BezierLine ---
                // Drive forward to (-39.0, 12.0)
                .setReversed(false)
                .strafeTo(new Vector2d(-39.0, 12.0))

                // --- Path7: BezierLine, reversed ---
                // Drive backwards to (-13.5, 13.0)
                .setReversed(true)
                .strafeTo(new Vector2d(-13.5, 13.0))

                .build();

        // Zero the turret motor on initialization
        Turret.INSTANCE.turret.zeroed();

        // Ensure turret is in command-based control mode (not manual tracking mode)
        Turret.INSTANCE.disableManualControl();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Start Pose", "%.1f, %.1f, %.1f°",
            startPose.position.x, startPose.position.y, Math.toDegrees(startPose.heading.toDouble()));
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }

    /**
     * Autonomous routine: Execute the 7-path sequence, then sweep the turret
     */
    private Command autonomousRoutine() {
        return new SequentialGroup(
                // Start the trajectory action
                new Command() {
                    @Override
                    public void start() {
                        actionRunning = true;
                    }

                    @Override
                    public void update() {
                        if (actionRunning) {
                            actionRunning = trajectoryAction.run(new com.acmerobotics.dashboard.telemetry.TelemetryPacket());
                        }
                    }

                    @Override
                    public boolean isDone() {
                        return !actionRunning;
                    }
                },

                // Wait 3 seconds after driving
                new Delay(3.0),

                // Turret sweep sequence: left -> right -> center
                Turret.INSTANCE.runTurret(-45.0),  // Turn left 45 degrees
                new Delay(3.0),  // 3 second wait

                Turret.INSTANCE.runTurret(45.0),   // Turn right 45 degrees
                new Delay(3.0),  // 3 second wait

                Turret.INSTANCE.runTurret(0.0)     // Return to center
        );
    }

    @Override
    public void onUpdate() {
        // Update telemetry with current robot pose during autonomous
        if (drive != null) {
            Pose2d currentPose = drive.localizer.getPose();
            telemetry.addData("Current Pose", "%.1f, %.1f, %.1f°",
                currentPose.position.x, currentPose.position.y, Math.toDegrees(currentPose.heading.toDouble()));
            telemetry.addData("Action Running", actionRunning);
            telemetry.update();
        }
    }
}
