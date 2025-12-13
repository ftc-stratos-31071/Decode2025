package org.firstinspires.ftc.teamcode.opmodes.teleops;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.IntakeSeqCmd;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;

/**
 * TestColor OpMode - Simple Ball Detection with Intake
 *
 * Detects if a ball is present based on distance threshold.
 *
 * Controls:
 * - Left Bumper: Toggle intake on/off
 * - Right Bumper: Hold for outtake (reverse)
 */
@Config
@TeleOp(name = "TestColor", group = "Testing")
public class TestColor extends NextFTCOpMode {

    private RevColorSensorV3 colorSensor;
    private boolean intakeRunning = false;

    // ===== TUNABLE VIA FTC DASHBOARD =====
    public static double DETECTION_DISTANCE_MM = 100.0;

    public TestColor() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE),
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "Color");

        try {
            ((LynxI2cDeviceSynch) colorSensor.getDeviceClient())
                    .setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
            telemetry.addLine("✓ Fast I2C mode enabled");
        } catch (Exception e) {
            telemetry.addLine("⚠ Could not set Fast I2C mode");
        }

        Intake.INSTANCE.defaultPos.schedule();

        telemetry.addLine("TestColor - Ball Detection");
        telemetry.addLine("Left Bumper: Toggle Intake");
        telemetry.addLine("Right Bumper: Hold for Outtake");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(() -> {
            if (intakeRunning) {
                Intake.INSTANCE.zeroPower.schedule();
                intakeRunning = false;
            } else {
                IntakeSeqCmd.create().schedule();
                intakeRunning = true;
            }
        });

        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> {
            Intake.INSTANCE.turnOnReverse.schedule();
            intakeRunning = false;
        });

        Gamepads.gamepad1().rightBumper().whenBecomesFalse(() -> {
            Intake.INSTANCE.zeroPower.schedule();
        });
    }

    @Override
    public void onUpdate() {
        double distanceMM = colorSensor.getDistance(DistanceUnit.MM);

        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        int red = (int) (colors.red * 255);
        int green = (int) (colors.green * 255);
        int blue = (int) (colors.blue * 255);

        float[] hsv = new float[3];
        Color.RGBToHSV(red, green, blue, hsv);

        boolean ballDetected = distanceMM < DETECTION_DISTANCE_MM;

        telemetry.addLine("===== BALL DETECTION =====");

        if (ballDetected) {
            telemetry.addData("STATUS", "✓ BALL DETECTED");
        } else {
            telemetry.addData("STATUS", "No ball");
        }
        telemetry.addLine();

        telemetry.addData("Distance", "%.1f mm", distanceMM);
        telemetry.addData("Threshold", "%.1f mm", DETECTION_DISTANCE_MM);
        telemetry.addLine();

        telemetry.addData("RGB", "%d, %d, %d", red, green, blue);
        telemetry.addData("Hue", "%.0f°", hsv[0]);
        telemetry.addLine();

        telemetry.addData("Intake", intakeRunning ? "RUNNING" : "STOPPED");

        telemetry.update();
    }
}
