package org.firstinspires.ftc.teamcode.opmodes.teleops;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.AutoAdjustShooterCmd;
import org.firstinspires.ftc.teamcode.commands.IntakeSeqCmd;
import org.firstinspires.ftc.teamcode.commands.RapidFireCmd;
import org.firstinspires.ftc.teamcode.constants.ShooterInterpolation;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.CameraDistanceCalculator;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

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
 * TeleOpAutoShooter - TeleOp with automatic distance-based shooter adjustment
 *
 * This demonstrates how ShooterInterpolation integrates with your robot:
 * - Press X: Auto-adjust shooter based on camera distance
 * - Press A: Shoot with current settings
 * - Turret still tracks AprilTags automatically
 *
 * Uses CameraDistanceCalculator to abstract camera hardware (ArduCam)
 */
@Config
@TeleOp(name = "TeleOpAutoShooter", group = "Demo")
public class TeleOpAutoShooter extends NextFTCOpMode {

    // Enable/disable auto shooter adjustment
    public static boolean AUTO_SHOOTER_ENABLED = true;

    // Target AprilTag ID for distance calculation
    public static int TARGET_TAG_ID = 20;

    private final MotorEx frontLeftMotor = new MotorEx("frontLeftMotor").brakeMode().reversed();
    private final MotorEx frontRightMotor = new MotorEx("frontRightMotor").brakeMode().reversed();
    private final MotorEx backLeftMotor = new MotorEx("backLeftMotor").brakeMode();
    private final MotorEx backRightMotor = new MotorEx("backRightMotor").brakeMode();

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private double currentDistance = -1.0;
    private boolean shooterOn = false;
    private boolean cameraInitialized = false;

    public TeleOpAutoShooter() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Turret.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        // Setup telemetry
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Initialize ArduCam with AprilTag processor
        cameraInitialized = initializeCamera();

        if (cameraInitialized) {
            dashboard.startCameraStream(visionPortal, 30);
        }

        // Initialize shooter
        Shooter.INSTANCE.stop();
        Turret.INSTANCE.setTurretAngleDeg(0.0);
        Turret.INSTANCE.goToAngle(0.0).schedule();

        telemetry.addData("Interpolation", ShooterInterpolation.getCalibrationStatus());
        telemetry.addData("Status", "Ready!");
        telemetry.update();
    }

    /**
     * Initialize camera hardware and configure it
     * @return true if camera was successfully initialized
     */
    private boolean initializeCamera() {
        try {
            aprilTag = new AprilTagProcessor.Builder()
                    .setLensIntrinsics(530.423, 530.423, 447.938, 335.553)
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .setDrawAxes(true)
                    .setDrawCubeProjection(true)
                    .setDrawTagOutline(true)
                    .build();

            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "arducam"))
                    .setCameraResolution(new Size(800, 600))
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .addProcessor(aprilTag)
                    .build();

            telemetry.addData("Camera", "✓ ArduCam Connected (AprilTag ID " + TARGET_TAG_ID + ")");
            return true;
        } catch (Exception e) {
            telemetry.addData("Camera", "✗ Not found - distance features disabled");
            telemetry.addData("Error", e.getMessage());
            return false;
        }
    }

    /**
     * Get the latest distance reading from the camera
     * @return Distance in inches, or -1 if no valid target
     */
    private double getCameraDistance() {
        if (!cameraInitialized || aprilTag == null) {
            return -1.0;
        }

        try {
            List<AprilTagDetection> detections = aprilTag.getDetections();

            // Look for the target AprilTag
            for (AprilTagDetection detection : detections) {
                if (detection.id == TARGET_TAG_ID) {
                    // Use CameraDistanceCalculator to handle the distance calculation
                    return CameraDistanceCalculator.calculateDistanceFromAprilTag(detection);
                }
            }

            return -1.0;  // No target tag found
        } catch (Exception e) {
            return -1.0;
        }
    }

    @Override
    public void onStartButtonPressed() {
        // Setup drive control
        var forward = Gamepads.gamepad1().leftStickY().negate();
        var strafe = Gamepads.gamepad1().leftStickX();
        var rotate = Gamepads.gamepad1().rightStickX();

        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor,
                forward, strafe, rotate
        );
        driverControlled.schedule();

        // ========== SHOOTER CONTROLS ==========

        // Right Bumper: Toggle shooter on/off
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> {
            shooterOn = !shooterOn;
            if (shooterOn) {
                // Start shooter at current interpolated settings
                if (currentDistance > 0) {
                    double rpm = ShooterInterpolation.getRPMForDistance(currentDistance);
                    Shooter.INSTANCE.runRPM(rpm).schedule();
                } else {
                    Shooter.INSTANCE.runRPM(3500).schedule();  // Default
                }
            } else {
                Shooter.INSTANCE.stop();
            }
        });

        // X Button: Auto-adjust shooter based on camera distance
        Gamepads.gamepad1().x().whenBecomesTrue(() -> {
            if (currentDistance > 0) {
                AutoAdjustShooterCmd.forDistance(currentDistance).schedule();
                telemetry.addData("Auto-Adjust", String.format("Set for %.1f inches", currentDistance));
            }
        });

        // A Button: Shoot
        Gamepads.gamepad1().a().whenBecomesTrue(() -> {
            Intake.INSTANCE.defaultPos().schedule();
            RapidFireCmd.create().schedule();
        });

        Gamepads.gamepad1().a().whenBecomesFalse(() -> {
            Intake.INSTANCE.zeroPowerIntake().schedule();
            Intake.INSTANCE.zeroPowerTransfer().schedule();
        });

        // Left Bumper: Intake
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(() -> {
            Intake.INSTANCE.moveServoPos().schedule();
            IntakeSeqCmd.create().schedule();
        });

        Gamepads.gamepad1().leftBumper().whenBecomesFalse(() -> {
            Intake.INSTANCE.zeroPowerIntake().schedule();
            Intake.INSTANCE.zeroPowerTransfer().schedule();
        });
    }

    @Override
    public void onUpdate() {
        // Get distance from camera using CameraDistanceCalculator
        currentDistance = getCameraDistance();

        // Auto-adjust shooter if enabled and shooter is on
        if (AUTO_SHOOTER_ENABLED && shooterOn && currentDistance > 0) {
            double rpm = ShooterInterpolation.getRPMForDistance(currentDistance);
            double hood = ShooterInterpolation.getHoodForDistance(currentDistance);

            Shooter.INSTANCE.setTargetRPM(rpm);
            Shooter.INSTANCE.setHood(hood).schedule();
        }

        // Get shooter status
        double currentRPM = Shooter.INSTANCE.getRPM();
        boolean atSpeed = Shooter.INSTANCE.atSpeed(50.0);

        // Telemetry
        telemetry.addData("═══ AUTO SHOOTER DEMO ═══", "");
        telemetry.addData("", "");
        telemetry.addData("Shooter", shooterOn ? "✓ ON" : "✗ OFF");
        telemetry.addData("Auto Adjust", AUTO_SHOOTER_ENABLED ? "✓ ENABLED" : "✗ DISABLED");
        telemetry.addData("Camera", cameraInitialized ? "✓ Active" : "✗ Unavailable");
        telemetry.addData("", "");

        if (currentDistance > 0) {
            double targetRPM = ShooterInterpolation.getRPMForDistance(currentDistance);
            double targetHood = ShooterInterpolation.getHoodForDistance(currentDistance);

            telemetry.addData("Distance", String.format("%.1f inches", currentDistance));
            telemetry.addData("Target RPM", String.format("%.0f", targetRPM));
            telemetry.addData("Target Hood", String.format("%.3f", targetHood));
            telemetry.addData("Current RPM", String.format("%.0f", currentRPM));
            telemetry.addData("At Speed?", atSpeed ? "✓ YES" : "✗ NO");
        } else {
            telemetry.addData("Distance", "No AprilTag " + TARGET_TAG_ID + " detected");
            telemetry.addData("Current RPM", String.format("%.0f", currentRPM));
        }

        telemetry.addData("", "");
        telemetry.addLine("═══ CONTROLS ═══");
        telemetry.addLine("Right Bumper: Shooter on/off");
        telemetry.addLine("X: Auto-adjust for current distance");
        telemetry.addLine("A: Shoot");
        telemetry.addLine("Left Bumper: Intake");

        telemetry.update();
    }

    @Override
    public void onStop() {
        Shooter.INSTANCE.stop();
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
