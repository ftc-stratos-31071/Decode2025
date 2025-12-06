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

        return new SequentialGroup(
                // First move servo to default position
                Intake.INSTANCE.defaultPos,
                // Then run intake reverse and shooter reverse in parallel
                new ParallelGroup(
                        Intake.INSTANCE.turnOnReverse,
                        Shooter.INSTANCE.moveShooter(-ShooterConstants.motorPower)  // Reverse
                ),
                new Delay(ShooterConstants.reverseDelaySeconds),
                Intake.INSTANCE.zeroPower,
                Shooter.INSTANCE.moveShooter(shooterPower)  // Use the passed parameter!
        );
    }
}
