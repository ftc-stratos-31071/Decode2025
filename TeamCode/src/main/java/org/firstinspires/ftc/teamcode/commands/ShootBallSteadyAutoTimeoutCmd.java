package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import dev.nextftc.core.commands.Command;

public class ShootBallSteadyAutoTimeoutCmd {

    /**
     * Custom auto shooter feed command with custom timeout.
     *
     * @param feedPower intake power while feeding
     * @param tolRpm shooter tolerance RPM before feeding starts
     * @param timeoutSec how long to run this command (seconds)
     */
    public static Command create(double feedPower, double tolRpm, double timeoutSec) {
        return new Command() {

            private double startTime;

            @Override
            public void start() {
                startTime = System.currentTimeMillis() / 1000.0;
            }

            @Override
            public void update() {
                if (Shooter.INSTANCE.atSpeed(tolRpm)) {
                    Intake.INSTANCE.setIntakePower(feedPower);
                } else {
                    Intake.INSTANCE.setIntakePower(0.0);
                }
            }

            @Override
            public boolean isDone() {
                double currentTime = System.currentTimeMillis() / 1000.0;
                return (currentTime - startTime) >= timeoutSec;
            }

            @Override
            public void stop(boolean interrupted) {
                Intake.INSTANCE.setIntakePower(0.0);
            }
        }.requires(Intake.INSTANCE);
    }

    /** Default behavior: your standard values + IntakeConstants.shootTimeCont */
    public static Command create() {
        return create(IntakeConstants.shootPower, 50, IntakeConstants.shootTimeCont);
    }
}