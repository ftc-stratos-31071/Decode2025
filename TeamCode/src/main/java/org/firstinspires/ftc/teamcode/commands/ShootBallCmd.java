package org.firstinspires.ftc.teamcode.commands;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;

import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.opmodes.autos.LaserRangefinder;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class ShootBallCmd {
    public static Command create(final LaserRangefinder cSensor) {
        return new SequentialGroup(
                ShootBallOne.create(),
                new Delay(IntakeConstants.sequenceDelay),
                Intake.INSTANCE.moveServoPos,
                Intake.INSTANCE.waitForBall(cSensor),
                new Delay(IntakeConstants.sequenceDelay),
                ShootBallOne.create(),
                new Delay(IntakeConstants.sequenceDelay),
                Intake.INSTANCE.moveServoPos,
                Intake.INSTANCE.waitForBall(cSensor),
                new Delay(IntakeConstants.sequenceDelay),
                ShootBallOne.create()
        );
    }
}