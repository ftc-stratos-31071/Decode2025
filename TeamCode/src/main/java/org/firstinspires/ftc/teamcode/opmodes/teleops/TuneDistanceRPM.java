package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

/**
 * TuneDistanceRPM
 *
 * Purpose: tune distance -> RPM mapping using Limelight AprilTag distance.
 * Hood is LOCKED at a constant position. You only tune RPM curve.
 *
 * Controls:
 *  - Right Bumper: enable shooter (distance-based RPM)
 *  - X: stop shooter
 *  - DPad Left/Right: adjust RPM_A
 *  - DPad Down/Up: adjust RPM_B
 *
 * Tune from FTC Dashboard:
 *  - CAM_HEIGHT_M, TAG_HEIGHT_M, CAM_PITCH_DEG, DISTANCE_SCALE
 *  - RPM_A, RPM_B, RPM_C
 *  - HOOD_LOCK_POS
 */
@Config
@TeleOp(name = "TuneDistanceRPM", group = "Tuning")
public class TuneDistanceRPM extends NextFTCOpMode {

    // ===== Drive (optional but useful to reposition while tuning) =====
    private final MotorEx frontLeftMotor  = new MotorEx("frontLeftMotor").brakeMode().reversed();
    private final MotorEx frontRightMotor = new MotorEx("frontRightMotor").brakeMode();
    private final MotorEx backLeftMotor   = new MotorEx("backLeftMotor").brakeMode().reversed();
    private final MotorEx backRightMotor  = new MotorEx("backRightMotor").brakeMode();

    // ===== Limelight =====
    private Limelight3A limelight;

    // ===== Hood lock =====
    public static boolean HOOD_LOCKED = true;
    public static double HOOD_LOCK_POS = 0.45; // tune once, then leave

    // ===== Distance math (tune until Distance(m) looks correct) =====
    public static double CAM_HEIGHT_M = 0.30;
    public static double TAG_HEIGHT_M = 1.20;
    public static double CAM_PITCH_DEG = 20.0;
    public static double DISTANCE_SCALE = 1.0;

    public static double MIN_DIST_M = 0.5;
    public static double MAX_DIST_M = 5.0;

    // Optional smoothing (helps noise)
    public static double DIST_SMOOTHING = 0.7; // 0..1 (higher = smoother, slower response)

    // ===== RPM curve =====
    // RPM(d) = RPM_A + RPM_B*d + RPM_C*d^2
    public static double RPM_A = 2600;
    public static double RPM_B = 700;
    public static double RPM_C = 0.0;

    public static double MIN_RPM = 1000;
    public static double MAX_RPM = 6000;

    // Quick bump tuning from gamepad
    public static double RPM_A_STEP = 50;
    public static double RPM_B_STEP = 25;

    // ===== Runtime state =====
    private boolean shooterEnabled = false;
    private double distanceM = Double.NaN;
    private double smoothedDistM = Double.NaN;
    private int chosenTagId = -1;

    public TuneDistanceRPM() {
        addComponents(
                new SubsystemComponent(Shooter.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Init Limelight
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(0);
            limelight.start();

            dashboard.startCameraStream(limelight, 0);
            telemetry.addData("Limelight", "✓ Connected");
        } catch (Exception e) {
            telemetry.addData("Limelight", "✗ ERROR: %s", e.getMessage());
        }

        telemetry.addLine("TuneDistanceRPM Ready");
        telemetry.addLine("RB = shooter on (distance RPM). X = stop.");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // Drive control
        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor,
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate()
        );
        driverControlled.schedule();

        // Lock hood at start
        if (HOOD_LOCKED) {
            Shooter.INSTANCE.moveServo(HOOD_LOCK_POS).schedule();
        }

        // Shooter enable/disable
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> shooterEnabled = true);
        Gamepads.gamepad1().x().whenBecomesTrue(() -> {
            shooterEnabled = false;
            Shooter.INSTANCE.stopShooter();
        });

        // Quick tuning bumps (optional)
        Gamepads.gamepad1().dpadLeft().whenBecomesTrue(() -> RPM_A -= RPM_A_STEP);
        Gamepads.gamepad1().dpadRight().whenBecomesTrue(() -> RPM_A += RPM_A_STEP);

        Gamepads.gamepad1().dpadDown().whenBecomesTrue(() -> RPM_B -= RPM_B_STEP);
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(() -> RPM_B += RPM_B_STEP);
    }

    @Override
    public void onUpdate() {
        // Re-lock hood continuously if you want it absolutely fixed (optional)
        if (HOOD_LOCKED) {
            Shooter.INSTANCE.moveServo(HOOD_LOCK_POS).schedule();
        }

        // Read Limelight
        LLResult result = null;
        if (limelight != null) {
            try {
                result = limelight.getLatestResult();
            } catch (Exception ignored) {}
        }

        chosenTagId = -1;
        distanceM = Double.NaN;

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                // Choose closest tag by (XPixels*YPixels) heuristic
                LLResultTypes.FiducialResult closest = null;
                double largest = 0.0;
                for (LLResultTypes.FiducialResult f : fiducials) {
                    double areaLike = f.getTargetXPixels() * f.getTargetYPixels();
                    if (closest == null || areaLike > largest) {
                        closest = f;
                        largest = areaLike;
                    }
                }

                if (closest != null) {
                    chosenTagId = closest.getFiducialId();
                    double ty = result.getTy();
                    distanceM = distanceMetersFromTy(ty);

                    if (!Double.isNaN(distanceM)) {
                        if (Double.isNaN(smoothedDistM)) smoothedDistM = distanceM;
                        smoothedDistM = DIST_SMOOTHING * smoothedDistM + (1.0 - DIST_SMOOTHING) * distanceM;
                    }
                }
            }
        }

        double useDist = !Double.isNaN(smoothedDistM) ? smoothedDistM : distanceM;
        double targetRpm = (!Double.isNaN(useDist)) ? rpmFromDistance(useDist) : 0.0;

        // Apply shooter setpoint if enabled AND we have distance
        if (shooterEnabled && targetRpm > 0) {
            Shooter.INSTANCE.setTargetRPM(targetRpm);
        } else if (!shooterEnabled) {
            // do nothing; X button stops shooter
        }

        // Telemetry
        double currentRpm = Shooter.INSTANCE.getRPM();

        telemetry.addLine("==== Distance → RPM Tuning ====");
        telemetry.addData("Shooter Enabled (RB)", shooterEnabled ? "YES" : "NO");
        telemetry.addData("Chosen Tag ID", chosenTagId);

        telemetry.addData("Distance Raw (m)", Double.isNaN(distanceM) ? "N/A" : String.format("%.2f", distanceM));
        telemetry.addData("Distance Smooth (m)", Double.isNaN(smoothedDistM) ? "N/A" : String.format("%.2f", smoothedDistM));

        telemetry.addData("RPM Model", String.format("RPM = %.0f + %.0f*d + %.2f*d^2", RPM_A, RPM_B, RPM_C));
        telemetry.addData("Target RPM", targetRpm > 0 ? String.format("%.0f", targetRpm) : "N/A");
        telemetry.addData("Current RPM", String.format("%.0f", currentRpm));
        telemetry.addData("Error", targetRpm > 0 ? String.format("%.0f", (targetRpm - currentRpm)) : "N/A");

        telemetry.addData("Hood Locked", HOOD_LOCKED ? "YES" : "NO");
        telemetry.addData("Hood Pos", String.format("%.2f", HOOD_LOCK_POS));

        telemetry.addLine();
        telemetry.addLine("Tune order: fix Distance → tune RPM_A midrange → tune RPM_B longrange → only then adjust RPM_C");
        telemetry.update();
    }

    // ===== Helpers =====
    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private double distanceMetersFromTy(double tyDeg) {
        double denom = Math.tan(Math.toRadians(CAM_PITCH_DEG + tyDeg));
        if (Math.abs(denom) < 1e-3) return Double.NaN;

        double d = (TAG_HEIGHT_M - CAM_HEIGHT_M) / denom;
        d *= DISTANCE_SCALE;
        return clamp(d, MIN_DIST_M, MAX_DIST_M);
    }

    private double rpmFromDistance(double dMeters) {
        double d = clamp(dMeters, MIN_DIST_M, MAX_DIST_M);
        double rpm = RPM_A + RPM_B * d + RPM_C * d * d;
        return clamp(rpm, MIN_RPM, MAX_RPM);
    }
}