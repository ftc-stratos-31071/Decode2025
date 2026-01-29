package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name="Basic Servo Control", group="Examples")
public class BasicServoControl extends LinearOpMode {

    private Servo myServo;

    @Override
    public void runOpMode() {

        // Initialize the servo from the hardware map
        myServo = hardwareMap.get(Servo.class, "HoodServo");

        // Set initial position
        myServo.setPosition(0.0); // 0 = one extreme, 1 = other extreme

        waitForStart();

        while (opModeIsActive()) {

            // Move servo to 0.0 when pressing 'a'
            if (gamepad1.a) {
                myServo.setPosition(0.0);
                telemetry.addData("Servo Position", "0.0");
            }

            // Move servo to 1.0 when pressing 'b'
            if (gamepad1.b) {
                myServo.setPosition(1.0);
                telemetry.addData("Servo Position", "1.0");
            }

            telemetry.update();
        }
    }
}