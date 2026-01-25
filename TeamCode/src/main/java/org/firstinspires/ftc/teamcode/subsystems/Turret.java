package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.FeedbackCRServoEx;

@Config
public class Turret implements Subsystem {

    public static final Turret INSTANCE = new Turret();

    /* ================= HARDWARE ================= */

    private final FeedbackCRServoEx pivotServo =
            new FeedbackCRServoEx(
                    0.01,
                    () -> ActiveOpMode.hardwareMap().analogInput.get("pivotAnalog"),
                    () -> ActiveOpMode.hardwareMap().crservo.get("pivotServo")
            );

    /* ================= PID CONSTANTS ================= */

    public static double kP = 4.0;
    public static double kI = 0.0;
    public static double kD = 0.2;

    public static double MAX_POWER = 1.0;
    public static double MIN_POWER = -1.0;

    /* ================= PID STATE ================= */

    private double targetAngle = 0.0;
    private boolean pidEnabled = false;

    private double integral = 0.0;
    private double lastError = 0.0;
    private long lastTimeNs = 0;

    /* ================= ANGLE TRACKING ================= */

    private double totalAngle = 0.0;
    private double previousAngle = 0.0;

    /* ================= DASHBOARD DEBUG ================= */

    public static double DEBUG_wrappedAngle;
    public static double DEBUG_totalAngle;
    public static double DEBUG_error;
    public static double DEBUG_output;

    private Turret() {
        lastTimeNs = System.nanoTime();
    }

    /* ================= PUBLIC API ================= */

    public void setTargetAngle(double radians) {
        targetAngle = radians;
        integral = 0.0;
        lastError = 0.0;
        lastTimeNs = System.nanoTime();
        pidEnabled = true;
    }

    public void stop() {
        pidEnabled = false;
        integral = 0.0;
        lastError = 0.0;
        pivotServo.setPower(0.0);
    }

    public boolean atTarget(double toleranceRad) {
        return Math.abs(targetAngle - totalAngle) <= toleranceRad;
    }

    public double getTotalAngle() {
        return totalAngle;
    }

    public double getWrappedAngle() {
        return pivotServo.getCurrentPosition();
    }

    /* ================= COMMANDS ================= */

    public Command goToAngle(double radians) {
        return new Command() {
            @Override
            public void start() {
                setTargetAngle(radians);
            }

            @Override
            public boolean isDone() {
                return false;
            }

            @Override
            public void stop(boolean interrupted) {}
        }.requires(this);
    }

    public Command openLoop(double power) {
        return new Command() {
            @Override
            public void start() {
                pidEnabled = false;
                pivotServo.setPower(power);
            }

            @Override
            public boolean isDone() {
                return true;
            }
        }.requires(this);
    }

    /* ================= MAIN CONTROL LOOP ================= */

    @Override
    public void periodic() {

        /* ---- Angle Unwrapping ---- */

        double currentAngle = pivotServo.getCurrentPosition();
        double delta = currentAngle - previousAngle;

        if (delta > Math.PI) delta -= 2 * Math.PI;
        else if (delta < -Math.PI) delta += 2 * Math.PI;

        totalAngle += delta;
        previousAngle = currentAngle;

        DEBUG_wrappedAngle = currentAngle;
        DEBUG_totalAngle = totalAngle;

        /* ---- PID ---- */

        if (!pidEnabled) return;

        long now = System.nanoTime();
        double dt = (now - lastTimeNs) / 1e9;
        lastTimeNs = now;

        if (dt <= 0) return;

        double error = targetAngle - totalAngle;
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double output =
                kP * error +
                        kI * integral +
                        kD * derivative;

        output = Math.max(MIN_POWER, Math.min(MAX_POWER, output));
        pivotServo.setPower(output);

        DEBUG_error = error;
        DEBUG_output = output;

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target Angle (rad)", targetAngle);
        packet.put("Total Angle (rad)", totalAngle);
        packet.put("Error", error);
        packet.put("Output", output);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}