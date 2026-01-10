package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.roadrunner.AutoMecanumDrive;

import dev.nextftc.core.commands.Command;

/**
 * Slowly drives forward (robot-forward direction) for a set amount of time.
 */
public class AutoSlowDriveForwardTimeoutCmd {

    /**
     * @param drive AutoMecanumDrive instance
     * @param forwardPower forward speed in RR units (try 0.1 to 0.3)
     * @param timeoutSec how long to creep forward (seconds)
     */
    public static Command create(AutoMecanumDrive drive, double forwardPower, double timeoutSec) {
        return new Command() {

            private double startTime;

            @Override
            public void start() {
                startTime = System.currentTimeMillis() / 1000.0;
            }

            @Override
            public void update() {
                // forward in robot frame (x = forward, y = strafe)
                drive.setDrivePowers(
                        new PoseVelocity2d(new Vector2d(forwardPower, 0.0), 0.0)
                );
            }

            @Override
            public boolean isDone() {
                double now = System.currentTimeMillis() / 1000.0;
                return (now - startTime) >= timeoutSec;
            }

            @Override
            public void stop(boolean interrupted) {
                // stop motion
                drive.setDrivePowers(
                        new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0)
                );
            }
        };
    }

    /** Default: forwardPower=0.2, timeoutSec=1.0 */
    public static Command create(AutoMecanumDrive drive) {
        return create(drive, 0.2, 1.0);
    }
}