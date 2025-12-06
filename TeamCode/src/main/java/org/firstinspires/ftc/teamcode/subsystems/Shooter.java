package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.constants.ShooterConstants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

@Config
public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();

    // Expansion Hub Servo Port 1
    private final ServoEx servo = new ServoEx("HoodServo");
    // Expansion Hub Motor Port 0
    private final MotorEx motor1 = new MotorEx("ShooterRight").brakeMode();
    // Expansion Hub Motor Port 1
    private final MotorEx motor2 = new MotorEx("ShooterLeft").brakeMode().reversed();

    // Velocity control system - rebuilt dynamically with current PID values
    private ControlSystem velocityController;

    private double targetVelocity = 0;
    public boolean velocityControlActive = false;  // Changed to public for debug access

    private Shooter() {
        // Initialize velocity controller with current values
        rebuildVelocityController();
    }

    /**
     * Rebuild the velocity controller with current PID values from ShooterConstants.
     * Call this when PID values change on the dashboard.
     */
    private void rebuildVelocityController() {
        velocityController = ControlSystem.builder()
                .velPid(ShooterConstants.velocityKp,
                        ShooterConstants.velocityKi,
                        ShooterConstants.velocityKd)
                .build();
    }

    // Dynamic servo commands - read values from ShooterConstants each time
    public final Command moveServoPos = new Command() {
        public void init() {
            servo.setPosition(ShooterConstants.servoPos);
        }

        @Override
        public boolean isDone() {
            return true;  // Completes immediately after setting position
        }
    }.requires(this);

    public final Command defaultPos = new Command() {
        public void init() {
            servo.setPosition(ShooterConstants.defaultPos);
        }

        @Override
        public boolean isDone() {
            return true;  // Completes immediately after setting position
        }
    }.requires(this);

    /**
     * Run shooter at a specific power (legacy method, not PID controlled)
     */
    public Command moveShooter(double shooterPower) {
        return new ParallelGroup(
                new SetPower(motor1, shooterPower),
                new SetPower(motor2, shooterPower)
        ).requires(this);
    }

    /**
     * Run shooter with PID velocity control at target RPM (battery-compensated)
     * This is the preferred method for consistent shooting
     */
    public Command runAtTargetRPM() {
        return runAtRPM(ShooterConstants.targetRPM);
    }

    /**
     * Run shooter with PID velocity control at specified RPM
     */
    public Command runAtRPM(double targetRPM) {
        return new ShooterVelocityCommand(targetRPM);
    }

    public Command moveServo(double servoPos) {
        return new SetPosition(servo, servoPos).requires(this);
    }

    // Dynamic commands that read from constants
    public final Command moveShooterReversed = new Command() {
        public void execute() {
            motor1.setPower(-ShooterConstants.motorPower);
            motor2.setPower(-ShooterConstants.motorPower);
        }

        @Override
        public boolean isDone() {
            return false;  // Runs continuously until interrupted
        }

        public void end(boolean interrupted) {
            // Don't stop motor here - let other commands control it
        }
    }.requires(this);

    public final Command zeroPower = new Command() {
        public void init() {
            motor1.setPower(ShooterConstants.zeroPower);
            motor2.setPower(ShooterConstants.zeroPower);
        }

        @Override
        public boolean isDone() {
            return true;  // Completes immediately after setting power
        }
    }.requires(this);

    /**
     * Get current shooter RPM
     */
    public double getRPM() {
        double ticksPerSecond = motor1.getVelocity();
        return (ticksPerSecond / 112.0) * 60.0;
    }

    /**
     * Check if shooter is at target velocity (within tolerance)
     */
    public boolean isAtTargetRPM() {
        double currentRPM = getRPM();
        return Math.abs(currentRPM - ShooterConstants.targetRPM) < ShooterConstants.rpmTolerance;
    }

    /**
     * Check if shooter is at a specific RPM (within tolerance)
     */
    public boolean isAtRPM(double targetRPM) {
        double currentRPM = getRPM();
        return Math.abs(currentRPM - targetRPM) < ShooterConstants.rpmTolerance;
    }

    @Override
    public void periodic() {
        // Apply velocity control if active
        if (velocityControlActive) {
            // Get current motor velocity in ticks/second
            double currentVelocity = motor1.getVelocity();

            // Calculate error
            double velocityError = targetVelocity - currentVelocity;

            // Calculate PID output (this is just the P, I, D correction terms)
            // The ControlSystem expects a KineticState with position and velocity
            double pidCorrection = velocityController.calculate(motor1.getState());

            // Calculate feedforward: power = Kf × targetVelocity
            // Kf is motor power per tick/sec, so this estimates the power needed
            double feedforward = targetVelocity * ShooterConstants.velocityKf;

            // Combine feedforward + PID correction
            double power = feedforward + pidCorrection;

            // Clamp to motor limits [-1, 1]
            power = Math.max(-1.0, Math.min(1.0, power));

            // Apply power to both motors
            motor1.setPower(power);
            motor2.setPower(power);
        }
    }

    /**
     * Custom command for velocity-controlled shooting
     */
    private class ShooterVelocityCommand extends Command {
        private final double targetTicksPerSecond;

        public ShooterVelocityCommand(double targetRPM) {
            this.targetTicksPerSecond = (targetRPM * 112.0) / 60.0;
            requires(Shooter.INSTANCE);
        }

        public void init() {
            // Rebuild velocity controller to pick up latest PID values from dashboard
            rebuildVelocityController();

            // Set target velocity and enable velocity control
            targetVelocity = targetTicksPerSecond;
            velocityControlActive = true;
            velocityController.reset();
        }

        public void execute() {
            // Velocity control happens in periodic()
            // Make sure velocityControlActive stays true
        }

        public void end(boolean interrupted) {
            // Disable velocity control when command ends
            velocityControlActive = false;
            motor1.setPower(0);
            motor2.setPower(0);
        }

        @Override
        public boolean isDone() {
            // This command runs continuously until interrupted
            return false;
        }
    }
}
