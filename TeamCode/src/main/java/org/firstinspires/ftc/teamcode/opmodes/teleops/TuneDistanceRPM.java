package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.IntakeSeqCmd;
import org.firstinspires.ftc.teamcode.commands.ShootBallCmd;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
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
 * TuneDistanceRPM (Minimal Controls)
 *
 * Purpose: tune distance -> RPM mapping using Limelight AprilTag distance.
 * Hood is LOCKED at a constant position. You only tune RPM curve.
 *
 * Controls (minimal):
 *  - Left Bumper:  Intake sequence (press to start, release to stop)
 *  - Right Bumper: Shooter ON (distance-based RPM)
 *  - A:            Shoot ball (kick)
 *  - X:            Shooter OFF
 *
 * Tune from FTC Dashboard:
 *  - CAM_HEIGHT_M, TAG_HEIGHT_M, CAM_PITCH_DEG, DISTANCE_SCALE
 *  - RPM_A, RPM_B, RPM_C
 *  - HOOD_LOCK_POS
 */
@Config
//@TeleOp(name = "TuneDistanceRPM", group = "Tuning")
public class TuneDistanceRPM extends NextFTCOpMode {

    // ===== Drive (useful while tuning) =====
    private final MotorEx frontLeftMotor  = new MotorEx("frontLeftMotor").brakeMode().reversed();
    private final MotorEx frontRightMotor = new MotorEx("frontRightMotor").brakeMode();
    private final MotorEx backLeftMotor   = new MotorEx("backLeftMotor").brakeMode().reversed();
    private final MotorEx backRightMotor  = new MotorEx("backRightMotor").brakeMode();

    // ===== Limelight =====
    private Limelight3A limelight;

    // ===== Hood lock =====
    public static boolean HOOD_LOCKED = true;
    public static double HOOD_LOCK_POS = 0.45; // tune once, then leave

    // ===== Distance math =====
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

    // ===== Runtime state =====
    private boolean shooterEnabled = false;
    private double distanceM = Double.NaN;
    private double smoothedDistM = Double.NaN;
    private int chosenTagId = -1;

    public TuneDistanceRPM() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE),
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
        telemetry.addLine("LB=intake  RB=shooter  A=shoot  X=stop shooter");
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

        // Put intake/shooter servos into known states
        Intake.INSTANCE.defaultPos.schedule();
        Shooter.INSTANCE.defaultPos.schedule();
        Shooter.INSTANCE.kickDefaultPos.schedule();

        // Lock hood at start
        if (HOOD_LOCKED) {
            Shooter.INSTANCE.moveServo(HOOD_LOCK_POS).schedule();
        }

        // ===== Minimal controls =====

        // LB: Intake sequence press/release
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(() -> IntakeSeqCmd.create().schedule());
        Gamepads.gamepad1().leftBumper().whenBecomesFalse(() -> Intake.INSTANCE.zeroPower.schedule());

        // RB: Shooter ON (distance RPM)
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> shooterEnabled = true);

        // X: Shooter OFF
        Gamepads.gamepad1().x().whenBecomesTrue(() -> {
            shooterEnabled = false;
            Shooter.INSTANCE.stopShooter();
        });

        // A: Shoot
//        Gamepads.gamepad1().a().whenBecomesTrue(() -> ShootBallCmd.create().schedule());
    }

    @Override
    public void onUpdate() {
        // Keep hood locked (optional but keeps it from drifting)
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
            // do nothing; X stops shooter
        }

        // Telemetry
        double currentRpm = Shooter.INSTANCE.getRPM();

        telemetry.addLine("==== Distance → RPM Tuning (Minimal) ====");
        telemetry.addData("Controls", "LB=intake  RB=shooter  A=shoot  X=stop shooter");

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
        telemetry.addLine("Tune: DISTANCE_SCALE first → RPM_A midrange → RPM_B longrange → RPM_C only if needed");
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