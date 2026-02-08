package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import dev.nextftc.core.commands.Command;

public class ShootBallSteadyCmdFar {

    private enum FeedState {
        IDLE,
        PRE_FEED,
        WAIT,
        FULL_FEED
    }

    public static Command create(double feedPower, double tolRpm) {
        return new Command() {

            private FeedState state = FeedState.IDLE;
            private long stateStartTime;

            private double timeInState() {
                return (System.nanoTime() - stateStartTime) / 1e9;
            }

            private void setState(FeedState newState) {
                state = newState;
                stateStartTime = System.nanoTime();
            }

            @Override
            public void start() {
                setState(FeedState.IDLE);
            }

            @Override
            public void update() {

                if (!Shooter.INSTANCE.atSpeed(tolRpm)) {
                    Intake.INSTANCE.setIntakePower(0.0);
                    Intake.INSTANCE.setTransferPower(0.0);
                    setState(FeedState.IDLE);
                    return;
                }

                switch (state) {

                    case IDLE:
                        Intake.INSTANCE.setIntakePower(0.0);
                        Intake.INSTANCE.setTransferPower(0.0);
                        break;

                    case PRE_FEED:
                        Intake.INSTANCE.setIntakePower(0.0);
                        Intake.INSTANCE.setTransferPower(feedPower);

                        if (timeInState() >= 0.5) {
                            setState(FeedState.WAIT);
                        }
                        break;

                    case WAIT:
                        Intake.INSTANCE.setIntakePower(0.0);
                        Intake.INSTANCE.setTransferPower(0.0);

                        if (timeInState() >= 0.5) {
                            setState(FeedState.FULL_FEED);
                        }
                        break;

                    case FULL_FEED:
                        Intake.INSTANCE.setIntakePower(feedPower);
                        Intake.INSTANCE.setTransferPower(feedPower);
                        break;
                }
            }

            @Override
            public boolean isDone() {
                return false;
            }

            @Override
            public void stop(boolean interrupted) {
                Intake.INSTANCE.setIntakePower(0.0);
                Intake.INSTANCE.setTransferPower(0.0);
            }
        }.requires(Intake.INSTANCE);
    }

    public static Command create() {
        return create(IntakeConstants.shootPower, 25);
    }
}