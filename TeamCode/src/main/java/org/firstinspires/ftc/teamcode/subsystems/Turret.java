package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.constants.TurretConstants;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;

@Config
public class Turret implements Subsystem {

    public static final Turret INSTANCE = new Turret();

    private Turret() {}

    private final ServoEx leftServo = new ServoEx("TurretLeft");
    private final ServoEx rightServo = new ServoEx("TurretRight");

    private double targetTurretDeg = TurretConstants.DEFAULT_TURRET_DEG;

    // TUNABLE: Set to true if turret rotates in wrong direction
    public static boolean REVERSE_DIRECTION = true;

    public void setTurretAngleDeg(double turretDeg) {
        // Convert logical angle (relative to center) to physical angle
        // Logical 0° → Physical 90° (actual center)
        double physicalAngle = turretDeg + TurretConstants.PHYSICAL_CENTER;

        // Clamp to safe physical range
        double minPhysical = TurretConstants.MIN_TURRET_DEG + TurretConstants.PHYSICAL_CENTER;
        double maxPhysical = TurretConstants.MAX_TURRET_DEG + TurretConstants.PHYSICAL_CENTER;

        targetTurretDeg = Math.max(minPhysical, Math.min(maxPhysical, physicalAngle));
    }

    private void updateServoPosition(double physicalAngleDeg) {
        // Physical range: center at 90°, range from -30° to 210°
        double minPhysical = TurretConstants.MIN_TURRET_DEG + TurretConstants.PHYSICAL_CENTER;
        double maxPhysical = TurretConstants.MAX_TURRET_DEG + TurretConstants.PHYSICAL_CENTER;
        double rangeSpan = maxPhysical - minPhysical;

        // Map physical angle to servo position [0, 1]
        double servoPos = (physicalAngleDeg - minPhysical) / rangeSpan;
        servoPos = Math.max(0, Math.min(1, servoPos));

        // Reverse direction if needed (turret rotating wrong way)
        if (REVERSE_DIRECTION) {
            servoPos = 1.0 - servoPos;
        }

        leftServo.setPosition(servoPos);
        rightServo.setPosition(servoPos);
    }

    @Override
    public void periodic() {
        // Instantly move turret to target position (no rate limiting)
        updateServoPosition(targetTurretDeg);
    }

    public double getTargetTurretDeg() {
        return targetTurretDeg;
    }

    public double getCurrentTurretDeg() {
        return targetTurretDeg; // Now always equals target since no rate limiting
    }

    public Command goToAngle(double turretDeg) {
        return new Command() {
            @Override
            public void start() {
                setTurretAngleDeg(turretDeg);
            }

            @Override
            public boolean isDone() {
                return true;
            }
        }.requires(this);
    }

    public Command defaultPosition() {
        return new Command() {
            @Override
            public void start() {
                setTurretAngleDeg(TurretConstants.DEFAULT_TURRET_DEG);
            }

            @Override
            public boolean isDone() {
                return true;
            }
        }.requires(this);
    }
}