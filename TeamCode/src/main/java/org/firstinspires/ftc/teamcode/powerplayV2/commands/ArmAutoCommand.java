package org.firstinspires.ftc.teamcode.powerplayV2.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.powerplayV2.subsystems.ArmSubsystem;

import java.util.function.IntSupplier;

public class ArmAutoCommand extends CommandBase {
    private ArmSubsystem arm;
    private IntSupplier index;

    public ArmAutoCommand(ArmSubsystem armSub, IntSupplier index) {
        this.arm = armSub;
        this.index = index;

        addRequirements(armSub);
    }

    @Override
    public void initialize() {
        arm.setAutonomousPosition(index.getAsInt());
    }

    @Override
    public boolean isFinished() {
        return true ;
    }
}
