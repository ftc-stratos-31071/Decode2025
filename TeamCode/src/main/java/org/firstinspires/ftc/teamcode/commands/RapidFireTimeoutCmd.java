package org.firstinspires.ftc.teamcode.commands;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;

import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * Wraps RapidFire shooting command and forces it to end after timeoutMs.
 * If the shooting finishes first, this finishes immediately.
 */
public class RapidFireTimeoutCmd {

    /**
     * Creates a RapidFire command with a timeout
     * @param timeoutMs Maximum time to run the shooting sequence (milliseconds)
     */
    public static Command create(double timeoutMs) {
        // Create the shooting sequence
        Command shootingSequence = new SequentialGroup(
//                WaitCmd.create(0.25),
                ShootBallSteadyCmd.create(IntakeConstants.shootPower, 100)
        );

        // Wrap it with timeout logic (convert ms to seconds)
        double timeoutSec = timeoutMs / 1000.0;

        return new Command() {
            private double startTime;
            private boolean timedOut = false;

            @Override
            public void start() {
                startTime = System.currentTimeMillis() / 1000.0;
                timedOut = false;
                shootingSequence.start();
            }

            @Override
            public void update() {
                shootingSequence.update();
            }

            @Override
            public boolean isDone() {
                double now = System.currentTimeMillis() / 1000.0;
                timedOut = (now - startTime) >= timeoutSec;

                // Check if wrapped command is done
                boolean cmdDone;
                try {
                    cmdDone = shootingSequence.isDone();
                } catch (Exception e) {
                    cmdDone = false;
                }

                // End if either:
                // 1) the shooting sequence ends normally
                // 2) we timed out
                return timedOut || cmdDone;
            }

            @Override
            public void stop(boolean interrupted) {
                // If we timed out, treat it like an interruption
                shootingSequence.stop(interrupted || timedOut);
            }
        }.requires(Intake.INSTANCE);  // Require Intake subsystem
    }

    /**
     * Default 1000ms (1 second) timeout
     */
    public static Command create() {
        return create(1000);
    }
}
