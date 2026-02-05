package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Config
@TeleOp(name = "TurretTester")
public class TurretTester extends NextFTCOpMode {

    // Tunable constraints via FTC Dashboard
    public static double MIN_TURRET_DEG = -30.0;
    public static double MAX_TURRET_DEG = 200.0;
    public static double SWEEP_SPEED = 50.0; // degrees per second
    public static double INIT_TURRET_DEG = 90.0; // Initial position

    private double currentTarget = 180.0; // Start at center
    private boolean movingToMax = true; // Start by moving toward max
    private long lastUpdateTime = 0;
    private double lastInitPosition = 180.0; // Track last init position for live updates

    public TurretTester() {
        addComponents(
                new SubsystemComponent(Turret.INSTANCE)
        );
    }

    @Override
    public void onInit() {
        // Initialize turret to our custom init position (ignore TurretConstants)
        currentTarget = INIT_TURRET_DEG;
        lastInitPosition = INIT_TURRET_DEG;

        // Set turret directly to our position
        Turret.INSTANCE.setTurretAngleDeg(currentTarget);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Min Constraint", MIN_TURRET_DEG);
        telemetry.addData("Max Constraint", MAX_TURRET_DEG);
        telemetry.addData("Sweep Speed", SWEEP_SPEED);
        telemetry.addData("Init Position", INIT_TURRET_DEG);
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        lastUpdateTime = System.currentTimeMillis();
        telemetry.addData("Status", "Running - Turret will sweep between constraints");
        telemetry.update();
    }

    @Override
    public void onUpdate() {
        long currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - lastUpdateTime) / 1000.0; // Convert to seconds
        lastUpdateTime = currentTime;

        // Clamp current target if constraints changed via dashboard
        if (currentTarget < MIN_TURRET_DEG) {
            currentTarget = MIN_TURRET_DEG;
            movingToMax = true;
        }
        if (currentTarget > MAX_TURRET_DEG) {
            currentTarget = MAX_TURRET_DEG;
            movingToMax = false;
        }

        // Calculate how much to move based on time and speed
        double deltaAngle = SWEEP_SPEED * deltaTime;

        // Update target position
        if (movingToMax) {
            currentTarget += deltaAngle;
            if (currentTarget >= MAX_TURRET_DEG) {
                currentTarget = MAX_TURRET_DEG;
                movingToMax = false; // Switch direction
            }
        } else {
            currentTarget -= deltaAngle;
            if (currentTarget <= MIN_TURRET_DEG) {
                currentTarget = MIN_TURRET_DEG;
                movingToMax = true; // Switch direction
            }
        }

        // Command turret to move to new position
        Turret.INSTANCE.goToAngle(currentTarget).schedule();

        // Telemetry
        telemetry.addData("Current Target", "%.1f°", currentTarget);
        telemetry.addData("Direction", movingToMax ? "→ MAX" : "← MIN");
        telemetry.addData("Min Constraint", MIN_TURRET_DEG);
        telemetry.addData("Max Constraint", MAX_TURRET_DEG);
        telemetry.addData("Sweep Speed", SWEEP_SPEED);
        telemetry.addData("Init Position", INIT_TURRET_DEG);
        telemetry.addData("Actual Turret Deg", Turret.INSTANCE.getTargetTurretDeg());
        telemetry.update();

        // Update INIT_TURRET_DEG live if changed from dashboard
        if (INIT_TURRET_DEG != lastInitPosition) {
            lastInitPosition = INIT_TURRET_DEG;
            currentTarget = INIT_TURRET_DEG;
            Turret.INSTANCE.goToAngle(currentTarget).schedule();
            telemetry.addData("Init Position Updated", INIT_TURRET_DEG);
        }
    }
}
