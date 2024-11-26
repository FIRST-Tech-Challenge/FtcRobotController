package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.UppiesSubsystem;

import java.util.function.BooleanSupplier;

public class SetUppiesCommand extends CommandBase {

    UppiesSubsystem uppiesSubsystem;
    UppiesSubsystem.UppiesState state;
    BooleanSupplier buttonPressedSupplier = null;

    public SetUppiesCommand(UppiesSubsystem uppiesSubsystem, UppiesSubsystem.UppiesState state) {
        super();
        this.uppiesSubsystem = uppiesSubsystem;
        this.state = state;
        addRequirements(uppiesSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        uppiesSubsystem.setUppiesState(state);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
