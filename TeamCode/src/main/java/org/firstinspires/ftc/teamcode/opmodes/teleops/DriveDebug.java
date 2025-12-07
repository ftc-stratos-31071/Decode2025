package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "DriveDebug", group = "Debug")
public class DriveDebug extends OpMode {

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    // 0 = FL, 1 = FR, 2 = BL, 3 = BR
    private int selected = 0;

    @Override
    public void init() {
        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeft   = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRight  = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("=== DriveDebug Initialized ===");
        telemetry.addLine("A = Front Left");
        telemetry.addLine("B = Front Right");
        telemetry.addLine("X = Back Left");
        telemetry.addLine("Y = Back Right");
        telemetry.addLine("DPad Up/Down = Forward/Backward");
        telemetry.addLine("DPad Left/Right = Left/Right");
        telemetry.update();
    }

    @Override
    public void loop() {

        // SELECT MOTOR
        if (gamepad1.a) selected = 0;
        if (gamepad1.b) selected = 1;
        if (gamepad1.x) selected = 2;
        if (gamepad1.y) selected = 3;

        // DETERMINE POWER
        double power = 0.0;

        if (gamepad1.dpad_up) {
            power = 0.5;
        } else if (gamepad1.dpad_down) {
            power = -0.5;
        } else if (gamepad1.dpad_left) {
            power = -0.5;
        } else if (gamepad1.dpad_right) {
            power = 0.5;
        }

        // APPLY POWER — only selected motor
        frontLeft.setPower(selected == 0 ? power : 0.0);
        frontRight.setPower(selected == 1 ? power : 0.0);
        backLeft.setPower(selected == 2 ? power : 0.0);
        backRight.setPower(selected == 3 ? power : 0.0);

        // TELEMETRY
        String motorName =
                selected == 0 ? "Front Left" :
                        selected == 1 ? "Front Right" :
                                selected == 2 ? "Back Left" :
                                        "Back Right";

        telemetry.addLine("=== Drive Debug ===");
        telemetry.addData("Selected Motor", motorName);
        telemetry.addData("Power", power);
        telemetry.addLine("Controls:");
        telemetry.addLine(" A = Front Left");
        telemetry.addLine(" B = Front Right");
        telemetry.addLine(" X = Back Left");
        telemetry.addLine(" Y = Back Right");
        telemetry.addLine(" DPad Up = Forward");
        telemetry.addLine(" DPad Down = Backward");
        telemetry.addLine(" DPad Left = Left");
        telemetry.addLine(" DPad Right = Right");
        telemetry.update();
    }

    @Override
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}