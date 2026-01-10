package org.firstinspires.ftc.teamcode.commands;

import dev.nextftc.core.commands.Command;

public class WaitCmd {

    /** Wait for a set amount of time (seconds). Does nothing else. */
    public static Command create(double seconds) {
        return new Command() {

            private double startTime;

            @Override
            public void start() {
                startTime = System.currentTimeMillis() / 1000.0;
            }

            @Override
            public void update() {
                // Do nothing
            }

            @Override
            public boolean isDone() {
                double currentTime = System.currentTimeMillis() / 1000.0;
                return (currentTime - startTime) >= seconds;
            }

            @Override
            public void stop(boolean interrupted) {
                // Do nothing
            }
        };
    }

    /** Default wait = 1 second */
    public static Command create() {
        return create(1.0);
    }
}