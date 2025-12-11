package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.constants.ShooterConstants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

@Config
public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();

    // Expansion Hub Servo Port 1
    private final ServoEx servo = new ServoEx("HoodServo");
    private final ServoEx servo2 = new ServoEx("KickerServo");
    // Expansion Hub Motor Port 0
    private final MotorEx motor1 = new MotorEx("ShooterRight").brakeMode();
    // Expansion Hub Motor Port 1
    private final MotorEx motor2 = new MotorEx("ShooterLeft").brakeMode().reversed();

    // PIDF state
    private double targetRpm = 0.0;
    private double integral = 0.0;
    private double lastError = 0.0;
    private boolean pidfEnabled = false;  // When true, PIDF controls motors in periodic()

    private Shooter() {
    }

    // ========= PIDF CONTROL API =========

    /** Enable PIDF control and set target RPM */
    public void setTargetRPM(double rpm) {
        targetRpm = Math.max(0, rpm);
        pidfEnabled = true;
    }

    public double getTargetRPM() {
        return targetRpm;
    }

    /** Stop shooter and reset PIDF controller */
    public void stopShooter() {
        targetRpm = 0.0;
        integral = 0.0;
        lastError = 0.0;
        pidfEnabled = false;
        motor1.setPower(0);
        motor2.setPower(0);
    }

    /** Reset PIDF controller state (clears integral/derivative) without stopping */
    public void resetPIDF() {
        integral = 0.0;
        lastError = 0.0;
    }

    // ========= SERVO COMMANDS =========

    public final Command moveServoPos = new Command() {
        @Override
        public void start() {
            servo.setPosition(ShooterConstants.servoPos);
        }

        @Override
        public boolean isDone() {
            return true;
        }
    }.requires(this);

    public final Command defaultPos = new Command() {
        @Override
        public void start() {
            servo.setPosition(ShooterConstants.defaultPos);
        }

        @Override
        public boolean isDone() {
            return true;
        }
    }.requires(this);

    public final Command kick = new Command() {
        @Override
        public void start() {
            servo2.setPosition(ShooterConstants.kickerPos);
        }

        @Override
        public boolean isDone() {
            return true;
        }
    }.requires(this);

    public final Command kickDefaultPos = new Command() {
        @Override
        public void start() {
            servo2.setPosition(ShooterConstants.kickDefaultPos);
        }

        @Override
        public boolean isDone() {
            return true;
        }
    }.requires(this);

    public Command moveServo(double servoPos) {
        return new SetPosition(servo, servoPos).requires(this);
    }

    // ========= OPEN-LOOP POWER CONTROL (disables PIDF) =========

    /** Run shooter at a specific power (open-loop, disables PIDF) */
    public Command moveShooter(double shooterPower) {
        return new Command() {
            @Override
            public void start() {
                pidfEnabled = false;  // Disable PIDF when using open-loop
                motor1.setPower(shooterPower);
                motor2.setPower(shooterPower);
            }

            @Override
            public boolean isDone() {
                return true;
            }
        }.requires(this);
    }

    /**
     * Command to run shooter in PIDF mode at a target RPM.
     * This command runs continuously until interrupted, allowing periodic() to handle PIDF control.
     */
    public Command runAtRPM(double rpm) {
        return new Command() {
            @Override
            public void start() {
                setTargetRPM(rpm);
            }

            @Override
            public void update() {
                // PIDF control happens in periodic()
            }

            @Override
            public boolean isDone() {
                return false;  // Run forever until interrupted
            }

            @Override
            public void stop(boolean interrupted) {
                stopShooter();
            }
        }.requires(this);
    }

    public final Command moveShooterReversed = new Command() {
        @Override
        public void start() {
            pidfEnabled = false;
        }

        @Override
        public void update() {
            motor1.setPower(-ShooterConstants.motorPower);
            motor2.setPower(-ShooterConstants.motorPower);
        }

        @Override
        public boolean isDone() {
            return false;
        }

        @Override
        public void stop(boolean interrupted) {
        }
    }.requires(this);

    public final Command zeroPower = new Command() {
        @Override
        public void start() {
            stopShooter();
        }

        @Override
        public boolean isDone() {
            return true;
        }
    }.requires(this);

    // ========= SENSOR =========

    /** Get current shooter RPM */
    public double getRPM() {
        double ticksPerSecond = motor1.getVelocity();
        return (ticksPerSecond / 112.0) * 60.0;
    }

    // ========= MAIN PIDF LOOP (CALLED EVERY CYCLE BY NEXTFTC) =========

    @Override
    public void periodic() {
        // Only run PIDF loop if enabled
        if (!pidfEnabled) {
            return;
        }

        if (targetRpm <= 0) {
            motor1.setPower(0);
            motor2.setPower(0);
            integral = 0;
            lastError = 0;
            return;
        }

        double currentRpm = getRPM();
        double error = targetRpm - currentRpm;

        integral += error;
        double derivative = error - lastError;
        lastError = error;

        // Read PIDF gains from ShooterConstants (live tunable via Dashboard)
        double output =
                ShooterConstants.kF * targetRpm +
                ShooterConstants.kP * error +
                ShooterConstants.kI * integral +
                ShooterConstants.kD * derivative;

        // Clamp output to power limits from ShooterConstants
        output = Math.max(ShooterConstants.MIN_POWER, Math.min(ShooterConstants.MAX_POWER, output));

        motor1.setPower(output);
        motor2.setPower(output);
    }
}
