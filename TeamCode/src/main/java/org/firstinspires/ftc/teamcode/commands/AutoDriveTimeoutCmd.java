package org.firstinspires.ftc.teamcode.commands;

import dev.nextftc.core.commands.Command;

/**
 * Wraps any command (usually a drive/trajectory command) and forces it to end after timeoutSec.
 * If the wrapped command finishes first, this finishes immediately.
 * Works with both Kotlin commands (isDone property) and Java commands (isDone() method).
 */
public class AutoDriveTimeoutCmd {

    /**
     * @param driveCmd the command you want to run (ex: a FollowPath or RoadRunner trajectory command)
     * @param timeoutSec max time to allow it to run (seconds)
     */
    public static Command create(Command driveCmd, double timeoutSec) {
        return new Command() {

            private double startTime;
            private boolean timedOut = false;

            @Override
            public void start() {
                startTime = System.currentTimeMillis() / 1000.0;
                timedOut = false;
                driveCmd.start();
            }

            @Override
            public void update() {
                driveCmd.update();
            }

            @Override
            public boolean isDone() {
                double now = System.currentTimeMillis() / 1000.0;
                timedOut = (now - startTime) >= timeoutSec;

                // Check if wrapped command is done
                // This works with both Kotlin properties and Java methods
                boolean cmdDone;
                try {
                    cmdDone = driveCmd.isDone();
                } catch (Exception e) {
                    // Fallback in case there's an issue
                    cmdDone = false;
                }

                // End if either:
                // 1) the wrapped command ends normally
                // 2) we timed out
                return timedOut || cmdDone;
            }

            @Override
            public void stop(boolean interrupted) {
                // If we timed out, treat it like an interruption so the drive cmd stops itself
                driveCmd.stop(interrupted || timedOut);
            }
        };
    }
}