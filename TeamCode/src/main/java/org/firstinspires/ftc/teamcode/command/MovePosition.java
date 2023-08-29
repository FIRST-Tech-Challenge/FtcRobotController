package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.TestMotor;

public class MovePosition extends CommandBase {
    private final TestMotor testMotor;
    private final int mPosition;
    public MovePosition(TestMotor testMotor, int position) {
        this.testMotor = testMotor;
        mPosition = position;
        addRequirements(testMotor);
    }

    @Override
    public void initialize() {
        testMotor.configPosition();
        testMotor.setPosition(mPosition);
    }

    @Override
    public void execute() {
        testMotor.setSpeed(0.35);
    }

    @Override
    public void end(boolean interrupted) {
        testMotor.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
