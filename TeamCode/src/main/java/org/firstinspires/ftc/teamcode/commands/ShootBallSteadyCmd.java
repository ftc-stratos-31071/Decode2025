package org.firstinspires.ftc.teamcode.commands;

import dev.nextftc.core.commands.Command;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Strategy A: Only feed when shooter is at speed.
 */
public class ShootBallSteadyCmd extends Command {

    private final double feedPower;
    private final double tolRpm;

    public ShootBallSteadyCmd(double feedPower, double tolRpm) {
        this.feedPower = feedPower;
        this.tolRpm = tolRpm;

        requires(Intake.INSTANCE);
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
        return false; // hold-to-run
    }

    @Override
    public void stop(boolean interrupted) {
        Intake.INSTANCE.setIntakePower(0.0);
    }
}