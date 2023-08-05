package org.firstinspires.ftc.teamcode.Slidy_PPV2.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Slidy_PPV2.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Slidy_PPV2.subsystems.FrontSliderSubsystem;

import java.util.function.BooleanSupplier;

public class FrontSliderConeCommand extends CommandBase {
    private FrontSliderSubsystem frontSlider;
    private BooleanSupplier isConeDetected;
    private ArmSubsystem arm;

    public FrontSliderConeCommand(FrontSliderSubsystem frontSlider, BooleanSupplier isConeDetected,
                                  ArmSubsystem arm) {
        this.frontSlider = frontSlider;
        this.isConeDetected = isConeDetected;
        this.arm = arm;

        addRequirements(frontSlider, arm);
    }

    @Override
    public void initialize() {
        arm.setIntake();
    }

    @Override
    public void execute() {
        frontSlider.open(0.2);
    }

    @Override
    public void end(boolean interrupted) {
        frontSlider.stop();
    }

    @Override
    public boolean isFinished() {
        return isConeDetected.getAsBoolean();
    }
}
