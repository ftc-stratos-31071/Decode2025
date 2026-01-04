package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.constants.TurretConstants;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.controllable.RunToPosition;

@Config
public class Turret2 implements Subsystem {
    public static final Turret2 INSTANCE = new Turret2();
    private Turret2() {}

    // Control Hub Motor Port 1
    public final MotorEx turret = new MotorEx("TurretMotor");

    private final ControlSystem turretController = ControlSystem.builder()
            .posPid(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD)
            .build();

    private double targetClicks = 0.0;
    private boolean manualControl = false;

    public Command runTurret(double degrees) {
        double clicksPerDegree = 1360.0 / 360.0;
        double clicks = clicksPerDegree * degrees;

        // RunToPosition uses the control system to move to the target position
        return new RunToPosition(turretController, clicks).requires(this);
    }

    /**
     * Set the target position for the turret in degrees.
     * This does NOT schedule a command - use this for continuous control in TeleOp.
     */
    public void setTargetDegrees(double degrees) {
        double clicksPerDegree = 1360.0 / 360.0;
        targetClicks = clicksPerDegree * degrees;
        manualControl = true;
    }

    /**
     * Disable manual control mode. Call this when you want to use command-based control.
     * Commands will automatically disable manual control when they run.
     */
    public void disableManualControl() {
        manualControl = false;
    }

    @Override
    public void periodic() {
        if (manualControl) {
            // When in manual control mode, use simple proportional control
            double currentClicks = turret.getCurrentPosition();
            double error = targetClicks - currentClicks;
            double power = error * TurretConstants.kP;

            // Clamp power to [-1, 1]
            power = Math.max(-1.0, Math.min(1.0, power));

            turret.setPower(power);
        }
        // Note: When commands are running (manualControl = false),
        // the command system handles motor control automatically
    }

}