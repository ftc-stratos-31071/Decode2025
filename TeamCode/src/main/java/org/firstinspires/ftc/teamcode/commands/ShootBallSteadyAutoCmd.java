package org.firstinspires.ftc.teamcode.commands;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class ShootBallSteadyAutoCmd {

    public static Command create(double feedPower, double tolRpm) {
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
                return (currentTime - startTime) >= IntakeConstants.shootTimeCont; // stop after 1 second
            }

            @Override
            public void stop(boolean interrupted) {
                Intake.INSTANCE.setIntakePower(0.0);
            }
        }.requires(Intake.INSTANCE);
    }

    /** Default: shootPower + reasonable tolerance */
    public static Command create() {
        return create(IntakeConstants.shootPower, 50);
    }
}