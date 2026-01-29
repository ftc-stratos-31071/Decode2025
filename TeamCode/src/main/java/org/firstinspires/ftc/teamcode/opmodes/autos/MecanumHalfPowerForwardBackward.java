package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "MecanumHalfPowerForwardBackward")
public class MecanumHalfPowerForwardBackward extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotor.class, "backRightMotor");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        setMecanumPower(0.5, 0, 0);
        sleep(4000);

        setMecanumPower(-0.5, 0, 0);
        sleep(4000);

        stopDrive();
    }

    void setMecanumPower(double forward, double strafe, double rotate) {
        frontLeft.setPower(forward + strafe + rotate);
        frontRight.setPower(forward - strafe - rotate);
        backLeft.setPower(forward - strafe + rotate);
        backRight.setPower(forward + strafe - rotate);
    }

    void stopDrive() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}