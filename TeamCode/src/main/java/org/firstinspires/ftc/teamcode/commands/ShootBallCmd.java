package org.firstinspires.ftc.teamcode.commands;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;

import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class ShootBallCmd {
    public static Command create() {
        return new SequentialGroup(
                Intake.INSTANCE.defaultPos,
                Intake.INSTANCE.shoot,
                Intake.INSTANCE.turnOnReverse,
                Shooter.INSTANCE.kick,
                new Delay(IntakeConstants.sequenceDelay),
                Shooter.INSTANCE.kickDefaultPos,
                new Delay(IntakeConstants.shootDelay),
                Intake.INSTANCE.moveServoPos,
                new Delay(IntakeConstants.sequenceDelay),
                Intake.INSTANCE.defaultPos,
                Intake.INSTANCE.shoot,
                Shooter.INSTANCE.kick,
                new Delay(IntakeConstants.sequenceDelay),
                Shooter.INSTANCE.kickDefaultPos,
                Intake.INSTANCE.turnOnReverse,
                new Delay(IntakeConstants.shootDelay),
                Intake.INSTANCE.moveServoPos,
                new Delay(IntakeConstants.sequenceDelay),
                Intake.INSTANCE.defaultPos,
                Intake.INSTANCE.shoot,
                Shooter.INSTANCE.kick,
                new Delay(IntakeConstants.sequenceDelay),
                Shooter.INSTANCE.kickDefaultPos
        );
    }
}
