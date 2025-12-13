package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import dev.nextftc.core.subsystems.Subsystem;


public class MecanumDrive implements Subsystem {
    public static final MecanumDrive INSTANCE = new MecanumDrive();


    private MecanumDrive() {}
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private IMU imu;


    public void init(HardwareMap hardwareMap, IMU imu) {
        this.imu = imu;


        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotorEx.class, "backRightMotor");


        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);


        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(imuParams);
        imu.resetYaw();
    }


    public double getHeading() {
        if (imu == null) return 0.0;
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }


    public void resetHeading() {
        if (imu != null) {
            imu.resetYaw();
        }
    }


    public void setPowerRotation(double power) {
        power = Math.max(-1.0, Math.min(1.0, power));


        frontLeft.setPower(-power);
        backLeft.setPower(-power);
        frontRight.setPower(power);
        backRight.setPower(power);
    }


    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }


    public void setPowerMecanum(double forward, double strafe, double rotate) {
        double frontLeftPower = forward + strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backLeftPower = forward - strafe + rotate;
        double backRightPower = forward + strafe - rotate;


        double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));


        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }


        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }


    @Override
    public void periodic() {
        // No periodic updates needed for direct motor control
    }
}




