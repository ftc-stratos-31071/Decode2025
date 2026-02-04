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

    private static final double SERVO_RANGE_DEG = 355.0;
    public static double CENTER_OFFSET_DEG = 140.0;

    public void setTurretAngleDeg(double turretDeg) {
        targetTurretDeg = turretDeg;

        double servoAngleDeg = turretDeg + CENTER_OFFSET_DEG;
        double servoPos = servoAngleDeg / SERVO_RANGE_DEG;

        if (servoPos < 0) servoPos = 0;
        if (servoPos > 1) servoPos = 1;

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