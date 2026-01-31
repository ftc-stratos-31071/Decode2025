package org.firstinspires.ftc.teamcode.commands;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * Strategy A (Gated Feed Only):
 * Does NOT control the shooter motors.
 * Only feeds when the shooter is within tolerance of its target RPM.
 *
 * Use with your existing shooter toggle (right bumper).
 */
public class ShootBallSteadyCmd {

    /**
     * @param feedPower Intake power while feeding (usually IntakeConstants.shootPower)
     * @param tolRpm Allowed RPM error before feeding is enabled (start ~120)
     */
    public static Command create(double feedPower, double tolRpm) {
        return new Command() {

            @Override
            public void update() {
                if (Shooter.INSTANCE.atSpeed(tolRpm)) {
                    Intake.INSTANCE.defaultPos();
                    Intake.INSTANCE.setIntakePower(feedPower);
                    Intake.INSTANCE.setTransferPower(feedPower);
                } else {
                    Intake.INSTANCE.setIntakePower(0.0);
                    Intake.INSTANCE.setTransferPower(0.0);
                }
            }

            @Override
            public boolean isDone() {
                return false; // run until interrupted (button released)
            }

            @Override
            public void stop(boolean interrupted) {
                Intake.INSTANCE.setIntakePower(0.0);
                Intake.INSTANCE.setTransferPower(0.0);
            }
        }.requires(Intake.INSTANCE);
    }

    /** Default: shootPower + a reasonable starter tolerance */
    public static Command create() {
        return create(IntakeConstants.shootPower, 50);
    }
}

