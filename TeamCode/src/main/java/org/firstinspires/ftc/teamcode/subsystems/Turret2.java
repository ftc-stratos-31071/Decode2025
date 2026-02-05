package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;

/**
 * Turret2 - Clean rewrite of turret control for dual Axon servos.
 *
 * CONCEPTS:
 * - "Raw Angle" = The actual servo angle (0° to 355° for Axon servos)
 * - "Logical Angle" = User-friendly angle where 0° = turret facing forward
 * - PHYSICAL_CENTER_RAW = The raw servo angle when turret faces forward
 *
 * TUNING PROCESS:
 * 1. Run Turret2Tuner OpMode
 * 2. Use bumpers to manually find the position where turret faces straight forward
 * 3. Note that raw angle - that's your PHYSICAL_CENTER_RAW
 * 4. Update PHYSICAL_CENTER_RAW in this file
 * 5. Now logical 0° will correctly mean "facing forward"
 */
@Config
public class Turret2 implements Subsystem {

    public static final Turret2 INSTANCE = new Turret2();

    private Turret2() {}

    // ═══════════════════════════════════════════════════════════════════
    // HARDWARE
    // ═══════════════════════════════════════════════════════════════════
    private final ServoEx leftServo = new ServoEx("TurretLeft");
    private final ServoEx rightServo = new ServoEx("TurretRight");

    // ═══════════════════════════════════════════════════════════════════
    // TUNABLE CONSTANTS (adjust via FTC Dashboard)
    // ═══════════════════════════════════════════════════════════════════

    /** Total rotation range of the Axon servo in degrees */
    public static double SERVO_RANGE_DEG = 355.0;

    /**
     * The RAW servo angle (in degrees) when the turret faces straight forward.
     * TUNE THIS: Run Turret2Tuner, find where turret faces forward, use that value.
     */
    public static double PHYSICAL_CENTER_RAW = 240.0;  //final value

    /**
     * Maximum rotation from center (in logical degrees).
     * Turret can rotate from -MAX_ROTATION to +MAX_ROTATION.
     */
    public static double MAX_ROTATION = 120.0;

    /** Set to true if turret rotates opposite to expected direction */
    public static boolean REVERSE_DIRECTION = false;

    // ═══════════════════════════════════════════════════════════════════
    // STATE
    // ═══════════════════════════════════════════════════════════════════

    /** Current target in LOGICAL degrees (0 = forward, positive = left, negative = right) */
    private double targetLogicalDeg = 0.0;

    /** Current raw angle being sent to servos */
    private double currentRawDeg = PHYSICAL_CENTER_RAW;

    // ═══════════════════════════════════════════════════════════════════
    // CORE METHODS
    // ═══════════════════════════════════════════════════════════════════

    /**
     * Set turret angle in LOGICAL coordinates.
     * 0° = facing forward
     * Positive = rotate left
     * Negative = rotate right
     */
    public void setAngle(double logicalDeg) {
        // Clamp to allowed range
        targetLogicalDeg = Range.clip(logicalDeg, -MAX_ROTATION, MAX_ROTATION);

        // Convert logical to raw
        double rawDeg = logicalToRaw(targetLogicalDeg);

        // Clamp to physical servo limits
        currentRawDeg = Range.clip(rawDeg, 0.0, SERVO_RANGE_DEG);

        // Apply to servos
        applyRawAngle(currentRawDeg);
    }

    /**
     * Set turret directly using RAW servo angle (0-355°).
     * Use this for tuning only!
     */
    public void setRawAngle(double rawDeg) {
        currentRawDeg = Range.clip(rawDeg, 0.0, SERVO_RANGE_DEG);
        targetLogicalDeg = rawToLogical(currentRawDeg);
        applyRawAngle(currentRawDeg);
    }

    /**
     * Set turret directly using RAW servo angle with NO LIMITS.
     * Use this for finding physical limits during tuning!
     */
    public void setRawAngleNoLimits(double rawDeg) {
        currentRawDeg = rawDeg;
        targetLogicalDeg = rawToLogical(currentRawDeg);
        applyRawAngle(currentRawDeg);
    }

    private void applyRawAngle(double rawDeg) {
        // Convert raw degrees (0-355) to servo position (0.0-1.0)
        double servoPos = rawDeg / SERVO_RANGE_DEG;
        servoPos = Range.clip(servoPos, 0.0, 1.0);

        // Apply direction reversal if needed
        if (REVERSE_DIRECTION) {
            servoPos = 1.0 - servoPos;
        }

        leftServo.setPosition(servoPos);
        rightServo.setPosition(servoPos);
    }

    // ═══════════════════════════════════════════════════════════════════
    // COORDINATE CONVERSIONS
    // ═══════════════════════════════════════════════════════════════════

    /** Convert logical angle (0 = forward) to raw servo angle */
    private double logicalToRaw(double logicalDeg) {
        return PHYSICAL_CENTER_RAW + logicalDeg;
    }

    /** Convert raw servo angle to logical angle (0 = forward) */
    private double rawToLogical(double rawDeg) {
        return rawDeg - PHYSICAL_CENTER_RAW;
    }

    // ═══════════════════════════════════════════════════════════════════
    // GETTERS
    // ═══════════════════════════════════════════════════════════════════

    /** Get current target in logical degrees (0 = forward) */
    public double getTargetLogicalDeg() {
        return targetLogicalDeg;
    }

    /** Get current raw servo angle in degrees (0-355) */
    public double getCurrentRawDeg() {
        return currentRawDeg;
    }

    /** Get current servo position (0.0-1.0) */
    public double getServoPosition() {
        return currentRawDeg / SERVO_RANGE_DEG;
    }

    // ═══════════════════════════════════════════════════════════════════
    // COMMANDS
    // ═══════════════════════════════════════════════════════════════════

    /** Command to go to a logical angle */
    public Command goToAngle(double logicalDeg) {
        return new Command() {
            @Override
            public void start() {
                setAngle(logicalDeg);
            }

            @Override
            public boolean isDone() {
                return true;
            }
        }.requires(this);
    }

    /** Command to go to center (forward) position */
    public Command goToCenter() {
        return goToAngle(0.0);
    }

    /** Command to go to a raw servo angle (for tuning) */
    public Command goToRawAngle(double rawDeg) {
        return new Command() {
            @Override
            public void start() {
                setRawAngle(rawDeg);
            }

            @Override
            public boolean isDone() {
                return true;
            }
        }.requires(this);
    }

    // ═══════════════════════════════════════════════════════════════════
    // SUBSYSTEM INTERFACE
    // ═══════════════════════════════════════════════════════════════════

    @Override
    public void periodic() {
        // Servos are set immediately in setAngle/setRawAngle
        // No periodic update needed for basic control
    }
}
