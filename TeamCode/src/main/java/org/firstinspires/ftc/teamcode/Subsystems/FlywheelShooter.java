package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.Config.FlywheelShooterConfig.*;

public abstract class FlywheelShooter {
    public enum ShooterState {
        STALLED, CHARGING, READY, IDLE
    }

    protected ShooterState shooterState = ShooterState.STALLED;

    protected void setVelocity(double velocity) {
        double feedForward = (kV * velocity) + kS * Math.signum(velocity);
        double error = velocity - getVelocity();
        double feedBack = error * kP;

        applyPower(feedBack + feedForward);
    }

    abstract void applyPower(double velocity);
    abstract double getVelocity();

    protected boolean isShooterRpmReady() {
        return (SHOOTING_VELOCITY - getVelocity()) < RPM_THRESHOLD;
    }

    public ShooterState getShooterState() {
        return shooterState;
    }

    public void shooterStateMachine() {
        switch (shooterState) {
            case STALLED:
                setVelocity(STALLED_VELOCITY);
                break;
            case IDLE:
                setVelocity(IDLE_VELOCITY);
                break;
            case CHARGING:
                setVelocity(SHOOTING_VELOCITY);
                if (isShooterRpmReady()) {
                    shooterState = ShooterState.READY;
                }
                break;
            case READY:
                setVelocity(SHOOTING_VELOCITY);
                if (!isShooterRpmReady()) {
                    shooterState = ShooterState.CHARGING;
                }
                break;
        }
    }

    public void shoot() {
        shooterState = ShooterState.CHARGING;
    }

    public void idle() {
        shooterState = ShooterState.IDLE;
    }

    public void stop() {
        shooterState = ShooterState.STALLED;
    }
}
