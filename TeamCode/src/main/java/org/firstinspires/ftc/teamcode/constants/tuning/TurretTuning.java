package org.firstinspires.ftc.teamcode.constants.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.constants.TurretConstants;


@TeleOp(name = "TurretTuning")
public class TurretTuning extends OpMode {
    private PIDController controller;

    double clicksDegreeProportion = 1440.0/360.0;
    private DcMotorEx turretMotor;

    @Override
    public void init() {
        controller = new PIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
    }

    @Override
    public void loop() {
        controller.setPID(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);
        int turretPos = turretMotor.getCurrentPosition();
        double pid = controller.calculate(turretPos, TurretConstants.pos);
        double power = pid;
        turretMotor.setPower(power);
        telemetry.addData("target", TurretConstants.pos);
        telemetry.addData("pos", turretPos);
        telemetry.update();
    }
}