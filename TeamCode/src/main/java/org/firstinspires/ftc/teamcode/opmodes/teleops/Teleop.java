package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.IntakeSeqCmd;
import org.firstinspires.ftc.teamcode.commands.ShootBallCmd;
import org.firstinspires.ftc.teamcode.commands.ShooterOffCmd;
import org.firstinspires.ftc.teamcode.commands.ShooterOnCmd;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@Config
@TeleOp(name = "TeleOp")
public class Teleop extends NextFTCOpMode {
    public Teleop() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Turret.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private final MotorEx frontLeftMotor = new MotorEx("frontLeftMotor").brakeMode().reversed();
    private final MotorEx frontRightMotor = new MotorEx("frontRightMotor").brakeMode();
    private final MotorEx backLeftMotor = new MotorEx("backLeftMotor").brakeMode().reversed();
    private final MotorEx backRightMotor = new MotorEx("backRightMotor").brakeMode();

    Limelight3A limelight;
    double motorTargetX = 0.0;
    final double TURRET_LIMIT_DEG = 90.0;

    double servoPos = ShooterConstants.defaultPos;
    double shooterPower = 0.5;

    @Override
    public void onInit() {
        // Add dashboard telemetry and camera stream
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        Intake.INSTANCE.defaultPos.schedule();
        Shooter.INSTANCE.defaultPos.schedule();
        Turret.INSTANCE.turret.zeroed();

        // Initialize Limelight with REAL connection
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.start();
            limelight.pipelineSwitch(0);

            // Stream Limelight camera to dashboard
            dashboard.startCameraStream(limelight, 0);

            telemetry.addData("Limelight", "✓ Connected");
            telemetry.addData("Camera Stream", "✓ Active");
        } catch (Exception e) {
            telemetry.addData("Limelight", "✗ ERROR: " + e.getMessage());
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate()
        );
        driverControlled.schedule();

        Gamepads.gamepad1().leftBumper().whenBecomesTrue(IntakeSeqCmd.create());
        Gamepads.gamepad1().leftBumper().whenBecomesFalse(ShooterOffCmd.create());

        Gamepads.gamepad1().dpadRight().whenBecomesTrue(() -> {
            shooterPower = Math.min(1.0, shooterPower + 0.1);
        });

        Gamepads.gamepad1().dpadLeft().whenBecomesTrue(() -> {
            shooterPower = Math.max(0.0, shooterPower - 0.1);
        });

        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> {
            shooterStartTime = System.currentTimeMillis();
            shooterTiming = true;
            ShooterOnCmd.create(shooterPower).schedule();
        });

        Gamepads.gamepad1().a().whenBecomesTrue(Intake.INSTANCE.turnOn);
        Gamepads.gamepad1().a().whenBecomesFalse(Intake.INSTANCE.zeroPower);

        Gamepads.gamepad1().b().whenBecomesTrue(Shooter.INSTANCE.zeroPower);

        // servo increment 0.1 steps
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(() -> {
            servoPos = Math.min(1.0, servoPos - 0.1);
            Shooter.INSTANCE.moveServo(servoPos).schedule();
        });

        Gamepads.gamepad1().dpadDown().whenBecomesTrue(() -> {
            servoPos = Math.max(0.0, servoPos + 0.1);
            Shooter.INSTANCE.moveServo(servoPos).schedule();
        });
    }

    boolean hasRumbled = false;

    // TIMER VARIABLES
    private long shooterStartTime = 0;
    private boolean shooterTiming = false;

    @Override
    public void onUpdate() {
        double rpm = Shooter.INSTANCE.getRPM() * 5;
        double targetRPM = shooterPower * 6000;

        // Get REAL Limelight result
        LLResult result = null;
        if (limelight != null) {
            try {
                result = limelight.getLatestResult();
            } catch (Exception e) {
                telemetry.addData("Limelight Error", e.getMessage());
            }
        }

        boolean shooterReady = Math.abs(rpm - targetRPM) < (targetRPM * 0.05);

        // timing logic
        long spinUpTimeMs = 0;
        if (shooterTiming) {
            spinUpTimeMs = System.currentTimeMillis() - shooterStartTime;

            if (shooterReady) {
                shooterTiming = false;
            }
        }

        // rumble when ready
        if (shooterReady && !hasRumbled) {
            Gamepads.gamepad1().getGamepad().invoke().rumble(500);
            hasRumbled = true;
        }

        if (!shooterReady) {
            hasRumbled = false;
        }

        telemetry.addData("═══ SHOOTER ═══", "");
        telemetry.addData("Shooter RPM", String.format("%.0f", rpm));
        telemetry.addData("Target RPM", String.format("%.0f", targetRPM));
        telemetry.addData("Ready?", shooterReady ? "✓ YES" : "✗ NO");

        if (shooterTiming) {
            telemetry.addData("Spin-up Time (ms)", spinUpTimeMs);
        } else if (shooterReady) {
            telemetry.addData("Final Spin-up Time (ms)", spinUpTimeMs);
        }

        telemetry.addData("Shooter Power", String.format("%.1f", shooterPower));

        telemetry.addData("", "");
        telemetry.addData("═══ TRACKING ═══", "");

        // Simple AprilTag tracking with REAL data
        if (result != null && result.isValid()) {
            double tx = result.getTx();  // REAL horizontal offset from Limelight
            motorTargetX = tx * 1.25;
            motorTargetX = Math.max(-TURRET_LIMIT_DEG, Math.min(TURRET_LIMIT_DEG, motorTargetX));

            telemetry.addData("Target Detected", "✓ YES");
            telemetry.addData("TX (offset)", String.format("%.2f°", tx));
            telemetry.addData("Turret Target", String.format("%.2f°", motorTargetX));
            telemetry.addData("Direction", tx > 0 ? "→ RIGHT" : "← LEFT");
        } else {
            if (result != null) {
                telemetry.addData("Target Detected", "✗ NO");
                telemetry.addData("Limelight Status", "No AprilTag in view");
            } else {
                telemetry.addData("Target Detected", "✗ ERROR");
                telemetry.addData("Limelight Status", "No data received");
            }
            telemetry.addData("Turret Target", String.format("%.2f°", motorTargetX));
        }

        Turret.INSTANCE.runTurret(motorTargetX).schedule();

        telemetry.update();
    }
}
