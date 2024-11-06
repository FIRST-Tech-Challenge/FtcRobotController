package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

public class SetArmPosition extends CommandBase {

    private Arm arm;
    private Arm.ArmState armState;
    public SetArmPosition(Arm arm, Arm.ArmState state) {
        this.arm = arm;
        this.armState = state;
    }

    @Override
    public void execute() {
        this.arm.goToPos(this.armState);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
