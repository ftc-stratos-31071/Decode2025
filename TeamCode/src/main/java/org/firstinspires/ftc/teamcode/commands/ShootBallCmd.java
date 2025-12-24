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
                ShootBallOne.create(),
                new Delay(IntakeConstants.sequenceDelay),
                ShootBallOne.create(),
                new Delay(IntakeConstants.sequenceDelay),
                ShootBallEnd.create()
        );
    }
}
