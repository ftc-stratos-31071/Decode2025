package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;


import org.firstinspires.ftc.teamcode.constants.ShooterConstants;


import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;


@Config
public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();


    // Expansion Hub Servo Port 1
    private final ServoEx servo = new ServoEx("HoodServo");
    private final ServoEx servo2 = new ServoEx("KickerServo");


    // Use MotorEx for shooter motors (NextFTC pattern)
    // MotorEx automatically sets RUN_WITHOUT_ENCODER when power is set
    private final MotorEx motor1 = new MotorEx("ShooterRight").brakeMode();
    private final MotorEx motor2 = new MotorEx("ShooterLeft").brakeMode().reversed();


    // PIDF state
    private double targetRpm = 0.0;
    private double integral = 0.0;
    private double lastError = 0.0;
    private boolean pidfEnabled = false;  // When true, PIDF controls motors in periodic()


    // Debug values (visible in dashboard)
    public static double DEBUG_currentRpm = 0;
    public static double DEBUG_error = 0;
    public static double DEBUG_output = 0;
    public static double DEBUG_velocityRight = 0;
    public static double DEBUG_velocityLeft = 0;


    private Shooter() {
    }


    // ========= PIDF CONTROL API =========


    /** Enable PIDF control and set target RPM */
    public void setTargetRPM(double rpm) {
        targetRpm = Math.max(0, rpm);
        pidfEnabled = true;
        // Reset integral when starting fresh
        integral = 0.0;
        lastError = 0.0;
    }


    public double getTargetRPM() {
        return targetRpm;
    }


    /** Check if PIDF control is currently enabled */
    public boolean isPidfEnabled() {
        return pidfEnabled;
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
    };  // No .requires(this) - kicker is independent of shooter motors


    public final Command kickDefaultPos = new Command() {
        @Override
        public void start() {
            servo2.setPosition(ShooterConstants.kickDefaultPos);
        }


        @Override
        public boolean isDone() {
            return true;
        }
    };  // No .requires(this) - kicker is independent of shooter motors


    public Command moveServo(double servoPos) {
        return new Command() {
            @Override
            public void start() {
                servo.setPosition(servoPos);
            }


            @Override
            public boolean isDone() {
                return true;
            }
        };
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


    /** Get current shooter RPM - uses MotorEx velocity */
    public double getRPM() {
        // Get velocity from both motors
        double ticksPerSecond = Math.abs(motor1.getVelocity());


        // Convert to RPM: (ticks/sec) / (ticks/rev) * 60 *
        return (ticksPerSecond / 28.0) * 60.0;
    }


    public boolean atSpeed(double tolRpm) {
        return Math.abs(getRPM() - targetRpm) <= tolRpm;
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


        // Get velocity from motors
        double velocityRight = Math.abs(motor1.getVelocity());
        double velocityLeft = Math.abs(motor2.getVelocity());
        double currentRpm = getRPM();


        double error = targetRpm - currentRpm;


        // Accumulate integral
        integral += error;


        double derivative = error - lastError;
        lastError = error;


        // Compute output using ShooterConstants (live tunable!)
        double output = ShooterConstants.kF * targetRpm
                + ShooterConstants.kP * error
                + ShooterConstants.kI * integral
                + ShooterConstants.kD * derivative;


        // Anti-windup: If output is saturated, prevent further integral accumulation
        if (output >= ShooterConstants.MAX_POWER || output <= ShooterConstants.MIN_POWER) {
            integral -= error;
        }


        // Clamp output to power limits
        output = Math.max(ShooterConstants.MIN_POWER, Math.min(ShooterConstants.MAX_POWER, output));


        // Update debug values (visible in FTC Dashboard under "Shooter")
        DEBUG_currentRpm = currentRpm;
        DEBUG_error = error;
        DEBUG_output = output;
        DEBUG_velocityRight = velocityRight;
        DEBUG_velocityLeft = velocityLeft;


        // Send to dashboard graph
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Shooter_TargetRPM", targetRpm);
        packet.put("Shooter_CurrentRPM", currentRpm);
        packet.put("Shooter_Error", error);
        packet.put("Shooter_Output", output);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);


        // Apply power to motors
        motor1.setPower(output);
        motor2.setPower(output);
    }
}
