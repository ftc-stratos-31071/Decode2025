package org.firstinspires.ftc.teamcode.commands;

import dev.nextftc.core.commands.Command;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;

public class ShooterOnCmd {

    /**
     * Create shooter command using PIDF control at target RPM
     * Only controls the shooter - does NOT touch intake (so they can run simultaneously)
     * @param targetRpm Target RPM for the shooter (uses PIDF from ShooterConstants)
     */
    public static Command create(double targetRpm) {
        return Shooter.INSTANCE.runAtRPM(targetRpm);
    }

    /**
     * Create shooter command using default target RPM from ShooterConstants
     */
    public static Command create() {
        return create(ShooterConstants.closeTargetRPM);
    }
}
