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

    public static double MIN_TURRET_DEG = 140;   // lowest logical angle
    public static double MAX_TURRET_DEG = 320; // highest logical angle


    public void setTurretAngleDeg(double turretDeg) {
        // Clamp to safe range
        targetTurretDeg = Math.max(MIN_TURRET_DEG, Math.min(MAX_TURRET_DEG, turretDeg));

        double servoPos = targetTurretDeg / Turret.MAX_TURRET_DEG;
        servoPos = Math.max(0, Math.min(1, servoPos));

        leftServo.setPosition(servoPos);
        rightServo.setPosition(servoPos);
    }

    public double getTargetTurretDeg() {
        return targetTurretDeg;
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