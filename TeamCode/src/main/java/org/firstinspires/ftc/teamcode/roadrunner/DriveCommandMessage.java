package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;

public final class DriveCommandMessage {
    public long timestamp;
    public double forwardVel;
    public double forwardAccel;
    public double lateralVel;
    public double lateralAccel;
    public double angularVel;
    public double angularAccel;

    public DriveCommandMessage(PoseVelocity2dDual<Time> poseVelocity) {
        this.timestamp = System.nanoTime();
        this.forwardVel = poseVelocity.linearVel.x.get(0);
        this.forwardAccel = poseVelocity.linearVel.x.get(1);
        this.lateralVel = poseVelocity.linearVel.y.get(0);
        this.lateralAccel = poseVelocity.linearVel.y.get(1);
        this.angularVel = poseVelocity.angVel.get(0);
        this.angularAccel = poseVelocity.angVel.get(1);
    }
}

