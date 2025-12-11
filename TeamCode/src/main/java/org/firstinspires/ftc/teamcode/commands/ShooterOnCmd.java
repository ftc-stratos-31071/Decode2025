package org.firstinspires.ftc.teamcode.commands;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;

public class ShooterOnCmd {

    public static Command create(double shooterPower) {
        return new ParallelGroup(
                        Intake.INSTANCE.turnOnReverse,
                        Shooter.INSTANCE.moveShooter(shooterPower)
        );
    }
}
