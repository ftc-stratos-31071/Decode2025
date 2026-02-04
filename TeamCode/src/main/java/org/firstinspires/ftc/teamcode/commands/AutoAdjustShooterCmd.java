package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.constants.ShooterInterpolation;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import dev.nextftc.core.commands.Command;

/**
 * AutoAdjustShooterCmd - Automatically adjust shooter RPM and hood based on distance
 *
 * This command uses ShooterInterpolation to set the shooter to the optimal
 * settings for a given distance.
 *
 * USAGE EXAMPLES:
 *
 * 1. Set shooter for specific distance:
 *    AutoAdjustShooterCmd.forDistance(75.0).schedule();
 *
 * 2. Use with Limelight distance calculation:
 *    double distance = calculateDistanceFromLimelight(ty);
 *    AutoAdjustShooterCmd.forDistance(distance).schedule();
 *
 * 3. In autonomous:
 *    new SequentialGroup(
 *        AutoAdjustShooterCmd.forDistance(85.0),
 *        WaitCmd.create(1.0),  // Wait for shooter to spin up
 *        RapidFireCmd.create()
 *    )
 */
public class AutoAdjustShooterCmd {

    /**
     * Create a command that adjusts shooter for a specific distance
     *
     * @param distanceInches Distance to target in inches
     * @return Command that sets shooter RPM and hood angle
     */
    public static Command forDistance(double distanceInches) {
        return new Command() {
            private double targetRPM;
            private double targetHood;

            @Override
            public void start() {
                // Get interpolated values
                targetRPM = ShooterInterpolation.getRPMForDistance(distanceInches);
                targetHood = ShooterInterpolation.getHoodForDistance(distanceInches);

                // Apply to shooter
                Shooter.INSTANCE.setTargetRPM(targetRPM);
                Shooter.INSTANCE.setHood(targetHood).schedule();

                // Start shooter if not already running
                if (Shooter.INSTANCE.getRPM() < 100) {
                    Shooter.INSTANCE.runRPM(targetRPM).schedule();
                }
            }

            @Override
            public boolean isDone() {
                // Command finishes immediately after setting values
                return true;
            }
        }.requires(Shooter.INSTANCE);
    }

    /**
     * Create a command that continuously tracks and adjusts shooter based on distance
     * This version runs continuously and updates shooter settings in real-time
     *
     * @param distanceSupplier Function that returns current distance (e.g., from Limelight)
     * @return Command that continuously adjusts shooter
     */
    public static Command trackDistance(java.util.function.DoubleSupplier distanceSupplier) {
        return new Command() {
            @Override
            public void update() {
                // Get current distance
                double distance = distanceSupplier.getAsDouble();

                // Get interpolated values
                double targetRPM = ShooterInterpolation.getRPMForDistance(distance);
                double targetHood = ShooterInterpolation.getHoodForDistance(distance);

                // Apply to shooter
                Shooter.INSTANCE.setTargetRPM(targetRPM);
                Shooter.INSTANCE.setHood(targetHood).schedule();
            }

            @Override
            public boolean isDone() {
                // Runs forever until interrupted
                return false;
            }
        }.requires(Shooter.INSTANCE);
    }

    /**
     * Create a command that waits for shooter to reach target RPM
     * Use this after calling forDistance() to ensure shooter is ready before shooting
     *
     * @param toleranceRPM How close to target RPM is acceptable (default: 50)
     * @param timeoutSeconds Maximum time to wait (default: 2.0 seconds)
     * @return Command that waits for shooter to spin up
     */
    public static Command waitForSpinup(double toleranceRPM, double timeoutSeconds) {
        return new Command() {
            private double startTime;

            @Override
            public void start() {
                startTime = System.currentTimeMillis() / 1000.0;
            }

            @Override
            public boolean isDone() {
                // Check if at speed
                if (Shooter.INSTANCE.atSpeed(toleranceRPM)) {
                    return true;
                }

                // Check timeout
                double elapsed = System.currentTimeMillis() / 1000.0 - startTime;
                return elapsed >= timeoutSeconds;
            }
        }.requires(Shooter.INSTANCE);
    }

    /**
     * Convenience method: wait for spinup with default parameters
     */
    public static Command waitForSpinup() {
        return waitForSpinup(50.0, 2.0);
    }
}

