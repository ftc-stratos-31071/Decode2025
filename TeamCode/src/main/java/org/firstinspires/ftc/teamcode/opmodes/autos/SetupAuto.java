package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

/**
 * Setup Autonomous - Resets all mechanisms to default positions.
 *
 * Run this before matches to ensure everything is properly positioned:
 * - Turret: Centered (0 degrees)
 * - Intake servo: Default position (0.525)
 * - Hood servo: Default position (1.0)
 * - Kicker servo: Default position (0.0)
 * - All motors: Stopped
 *
 * This matches the initialization state from TeleOp.
 */
@Autonomous(name = "Setup", group = "Setup")
public class SetupAuto extends NextFTCOpMode {

    public SetupAuto() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Turret.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        telemetry.addData("Status", "Setup Auto Ready");
        telemetry.addData("", "Press START to reset all mechanisms to default positions");
        telemetry.addData("", "");
        telemetry.addData("Will Reset:", "");
        telemetry.addData("  Turret", "→ Center (0°)");
        telemetry.addData("  Intake Servo", "→ Default (0.525)");
        telemetry.addData("  Hood Servo", "→ Default (1.0)");
        telemetry.addData("  Kicker Servo", "→ Default (0.0)");
        telemetry.addData("  Motors", "→ Stopped");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // Stop all motors first
        Shooter.INSTANCE.stopShooter();
        Intake.INSTANCE.zeroPower.schedule();

        // Reset turret to center (0 degrees)
        Turret.INSTANCE.turret.zeroed();
        Turret.INSTANCE.setTargetDegrees(0.0);

        // Reset all servos to default positions
        Intake.INSTANCE.defaultPos.schedule();
        Shooter.INSTANCE.defaultPos.schedule();
        Shooter.INSTANCE.kickDefaultPos.schedule();

        // Create a sequential command to display status and complete
        Command statusCommand = new SequentialGroup(
                new Delay(0.5),  // Wait for servos to move
                new Command() {
                    @Override
                    public void start() {
                        telemetry.clearAll();
                        telemetry.addData("Status", "✓ RESET COMPLETE");
                        telemetry.addData("", "");
                        telemetry.addData("Turret", "✓ Centered");
                        telemetry.addData("Intake Servo", "✓ Default Position");
                        telemetry.addData("Hood Servo", "✓ Default Position");
                        telemetry.addData("Kicker Servo", "✓ Default Position");
                        telemetry.addData("Motors", "✓ Stopped");
                        telemetry.addData("", "");
                        telemetry.addData("Ready", "Robot is ready for TeleOp!");
                        telemetry.update();
                    }

                    @Override
                    public boolean isDone() {
                        return true;
                    }
                }
        );
        statusCommand.schedule();
    }

    @Override
    public void onUpdate() {
        // Keep telemetry updated
        telemetry.update();
    }
}
