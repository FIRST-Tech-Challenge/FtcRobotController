package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.TestMotor;

public class FlyWheel extends CommandBase {
    private TestMotor mShooter;
    private  double mFlyWheelRPM;
    public FlyWheel(TestMotor shooter, double flyWheelRPM) {
        mShooter = shooter;
        mFlyWheelRPM = flyWheelRPM;
        addRequirements(mShooter);
    }

    @Override
    public void initialize() {
        mShooter.configVelocity();
        mShooter.setFlyWheelState(TestMotor.FlyWheelState.SPINNINGUP);
    }

    @Override
    public void execute() {
        mShooter.flyWheel(mFlyWheelRPM);
        mShooter.isAtRPM(24.0);
    }

    @Override
    public void end(boolean interrupted) {
        mShooter.setFlyWheelState(TestMotor.FlyWheelState.OFF);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
