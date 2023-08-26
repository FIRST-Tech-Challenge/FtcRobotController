package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.subsystem.Arm;

public class ArmMovePosition extends CommandBase {
    private final Arm mArm;
    private final int mPosition;
    public ArmMovePosition (Arm arm, int position) {
        mArm = arm;
        mPosition = position;
        addRequirements(mArm);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double kp = 0.01 * (mArm.getPosition() - mPosition);
        mArm.setSpeed(Range.clip(-0.2 * kp, -0.35, 0.35));
    }

    @Override
    public void end(boolean interrupted) {
        mArm.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(mArm.getPosition() - mPosition) <= 3;
    }
}
