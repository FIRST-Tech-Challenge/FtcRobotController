package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSub;

public class MoveArmToPos extends CommandBase {
    private final ArmSub armSub;
    private int armPos;

    public MoveArmToPos(ArmSub armSub) {
        this.armSub = armSub;
    }

    public Command storePos(int armPos) {
        this.armPos = armPos;
        return null;
    }
    public void moveToPos() {
        armSub.setPos(armPos);
    }
}
