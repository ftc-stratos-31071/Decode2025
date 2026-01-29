package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Config
public class TestBareVel extends OpMode {
    private DcMotorEx driveMotor;

    @Override
    public void init() {
        //TODO: Set device name
        driveMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        // Direction doesn't matter here
    }

    @Override
    public void loop() {
        driveMotor.setPower(1.0);
        double velocity = driveMotor.getVelocity();
        double rpm = velocity * 2.1429;

        String status;
        if (Math.abs(5700 - rpm) > 100) {
            status = "BAD";
        } else {
            status = "GOOD";
        }
        TelemetryPacket packet = new TelemetryPacket();
        telemetry.addData("Status: ", status);
        telemetry.addData("Current RPM", rpm);
        packet.put("Status", status);
        packet.put("Current RPM", rpm);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        telemetry.update();
    }
}