package org.firstinspires.ftc.teamcode.commands;


import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;

import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class ShootBallCont {
    public static Command create() {
        // The shooting sequence acts as the deadline - when it's done, everything stops
        return new SequentialGroup(
                // Move servo to default position at start
                Intake.INSTANCE.defaultPos(),
                new Delay(IntakeConstants.sequenceDelay),

                // First kick
                KickCmd.create(),
                new Delay(IntakeConstants.sequenceDelay),

                // Second kick
                KickCmd.create(),
                new Delay(IntakeConstants.sequenceDelay),

                // Third kick
                KickCmd.create()
        ).asDeadline(
                // Intake runs continuously, but only while the deadline (shooting sequence) runs
                Intake.INSTANCE.moveIntake(IntakeConstants.intakePower)
        ).then(
                // Clean up: ensure intake is stopped
                Intake.INSTANCE.zeroPower()
        );
    }
}
