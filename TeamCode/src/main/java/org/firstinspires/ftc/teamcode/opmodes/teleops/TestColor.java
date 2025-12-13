package org.firstinspires.ftc.teamcode.opmodes.teleops;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.IntakeSeqCmd;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;

/**
 * TestColor OpMode - Simple Ball Detection
 *
 * Detects balls using color sensor distance and identifies color.
 * When distance is below threshold, a ball is detected.
 *
 * Controls:
 * - Left Bumper: Toggle intake on/off
 * - Right Bumper: Hold for outtake (reverse)
 */
@Config
@TeleOp(name = "TestColor", group = "Testing")
public class TestColor extends NextFTCOpMode {

    private RevColorSensorV3 colorSensor;

    // Intake state
    private boolean intakeRunning = false;

    // Ball detection state
    private boolean ballDetected = false;
    private String detectedColor = "NONE";

    // Ball counting
    private int ballCount = 0;
    private boolean wasDetected = false;  // For edge detection (ball entering/leaving)

    // ===== TUNABLE VIA FTC DASHBOARD =====
    public static double DETECTION_DISTANCE_MM = 30.0;  // Ball detected if distance < this
    public static double LED_GAIN = 2.0;  // Sensor LED gain (1-3 recommended)

    // Hue ranges (0-360 degrees)
    public static double PURPLE_HUE_MIN = 260.0;
    public static double PURPLE_HUE_MAX = 310.0;
    public static double GREEN_HUE_MIN = 80.0;
    public static double GREEN_HUE_MAX = 160.0;

    public TestColor() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE),
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        // Initialize color sensor
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");

        // Set LED gain for better color detection
        colorSensor.setGain((float) LED_GAIN);

        // Enable I2C Fast Mode (400kHz) for faster reads
        try {
            ((LynxI2cDeviceSynch) colorSensor.getDeviceClient())
                    .setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
            telemetry.addLine("✓ Fast I2C mode enabled");
        } catch (Exception e) {
            telemetry.addLine("⚠ Using standard I2C mode");
        }

        // Initialize intake to default position
        Intake.INSTANCE.defaultPos.schedule();

        telemetry.addLine("TestColor - Ball Detection");
        telemetry.addLine("Left Bumper: Toggle Intake");
        telemetry.addLine("Right Bumper: Hold for Outtake");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // Left Bumper - Toggle intake on/off
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(() -> {
            if (intakeRunning) {
                Intake.INSTANCE.zeroPower.schedule();
                intakeRunning = false;
            } else {
                ballCount = 0;  // Reset count when starting intake
                IntakeSeqCmd.create().schedule();
                intakeRunning = true;
            }
        });

        // Right Bumper - Outtake (hold to run)
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> {
            Intake.INSTANCE.turnOnReverse.schedule();
            intakeRunning = false;
        });

        Gamepads.gamepad1().rightBumper().whenBecomesFalse(() -> {
            Intake.INSTANCE.zeroPower.schedule();
        });

        // A Button - Reset ball count
        Gamepads.gamepad1().a().whenBecomesTrue(() -> {
            ballCount = 0;
        });
    }

    @Override
    public void onUpdate() {
        // Update LED gain if changed in dashboard
        colorSensor.setGain((float) LED_GAIN);

        // Read raw sensor data
        double distance = colorSensor.getDistance(DistanceUnit.MM);
        var colors = colorSensor.getNormalizedColors();

        // Convert to RGB 0-255 for HSV calculation
        int red = (int) (colors.red * 255);
        int green = (int) (colors.green * 255);
        int blue = (int) (colors.blue * 255);

        // Calculate HSV
        float[] hsv = new float[3];
        Color.RGBToHSV(red, green, blue, hsv);
        double hue = hsv[0];        // 0-360
        double saturation = hsv[1]; // 0-1
        double value = hsv[2];      // 0-1 (brightness)

        // ===== BALL DETECTION =====
        // Simple: if distance < threshold, ball is detected
        ballDetected = distance < DETECTION_DISTANCE_MM;

        // Identify color when ball is detected
        if (ballDetected) {
            detectedColor = identifyColor(hue);
        } else {
            detectedColor = "NONE";
        }

        // Count balls passing through (rising edge detection)
        if (ballDetected && !wasDetected) {
            // Ball just entered sensor range
            ballCount++;

            // Rumble controller
            try {
                Gamepads.gamepad1().getGamepad().invoke().rumble(200);
            } catch (Exception ignored) {}
        }
        wasDetected = ballDetected;

        // ===== TELEMETRY =====
        telemetry.addLine("===== BALL DETECTION =====");
        telemetry.addLine();

        // Detection status
        if (ballDetected) {
            telemetry.addData("STATUS", "✓ BALL DETECTED");
            telemetry.addData("COLOR", detectedColor);
        } else {
            telemetry.addData("STATUS", "No ball");
        }
        telemetry.addLine();

        // Raw sensor readings
        telemetry.addLine("----- RAW SENSOR DATA -----");
        telemetry.addData("Distance", "%.1f mm (threshold: %.1f)", distance, DETECTION_DISTANCE_MM);
        telemetry.addData("RGB", "R:%d G:%d B:%d", red, green, blue);
        telemetry.addData("Hue", "%.1f°", hue);
        telemetry.addData("Saturation", "%.2f", saturation);
        telemetry.addData("Brightness", "%.2f", value);
        telemetry.addLine();

        // Ball count
        telemetry.addLine("----- BALL COUNT -----");
        telemetry.addData("Balls Counted", ballCount);
        telemetry.addData("Intake", intakeRunning ? "RUNNING" : "STOPPED");
        telemetry.addLine();

        // Controls reminder
        telemetry.addLine("----- CONTROLS -----");
        telemetry.addData("Left Bumper", "Toggle Intake");
        telemetry.addData("Right Bumper", "Hold for Outtake");
        telemetry.addData("A Button", "Reset Count");

        telemetry.update();
    }

    /**
     * Identify ball color based on hue value
     */
    private String identifyColor(double hue) {
        if (hue >= PURPLE_HUE_MIN && hue <= PURPLE_HUE_MAX) {
            return "PURPLE";
        }
        if (hue >= GREEN_HUE_MIN && hue <= GREEN_HUE_MAX) {
            return "GREEN";
        }
        return "UNKNOWN";
    }
}
