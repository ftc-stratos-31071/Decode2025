package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Disabled
@TeleOp(name = "Velocity PID Tuner (Single Encoder)")
public class VelocityPIDTuner extends OpMode {

    public DcMotorEx flywheelMotorRight; // Encoder motor
    public DcMotorEx flywheelMotorLeft;  // Follower motor

    // Target velocities (ticks/sec)
    double highVelocity = 3500;
    double lowVelocity  = 3000;
    double curTargetVelocity = highVelocity;

    // PIDF coefficients
    double P = 0.0;
    double F = 0.0;

    // Tuning step sizes
    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};
    int stepIndex = 1;

    @Override
    public void init() {
        flywheelMotorRight = hardwareMap.get(DcMotorEx.class, "ShooterRight");
        flywheelMotorLeft  = hardwareMap.get(DcMotorEx.class, "ShooterLeft");

        // Right motor: velocity control
        flywheelMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Left motor: follower
        flywheelMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Apply initial PIDF
        PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotorRight.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidf
        );

        telemetry.addLine("Velocity PID Tuner Initialized");
    }

    @Override
    public void loop() {

        // Toggle target velocity
        if (gamepad1.yWasPressed()) {
            curTargetVelocity =
                    (curTargetVelocity == highVelocity)
                            ? lowVelocity
                            : highVelocity;
        }

        // Change tuning step size
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        // Tune F
        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        // Tune P
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }

        // Update PIDF
        PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotorRight.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                pidf
        );

        // Set velocity on encoder motor
        flywheelMotorRight.setVelocity(curTargetVelocity);

        // Follower motor mirrors right motor power
        flywheelMotorLeft.setPower(flywheelMotorRight.getPower());

        // Telemetry
        double curVelocity = flywheelMotorRight.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", "%.1f", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.1f", curVelocity);
        telemetry.addData("Error", "%.1f", error);

        telemetry.addLine("--------------------------------");

        telemetry.addData("P", "%.5f (D-Pad Up/Down)", P);
        telemetry.addData("F", "%.5f (D-Pad Left/Right)", F);
        telemetry.addData("Step Size", "%.5f (B)", stepSizes[stepIndex]);

        telemetry.update();
    }
}