package org.firstinspires.ftc.teamcode.powerplayV2;

import com.arcrobotics.ftclib.command.CommandBase;

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
//        frontSlider.increasePosition();
    }

    @Override
    public boolean isFinished() {
        return isConeDetected.getAsBoolean();
    }
}
