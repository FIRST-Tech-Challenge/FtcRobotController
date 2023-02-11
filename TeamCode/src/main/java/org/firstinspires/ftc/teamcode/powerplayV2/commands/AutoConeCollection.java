package org.firstinspires.ftc.teamcode.powerplayV2.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.powerplayV2.ArmSubsystem;
import org.firstinspires.ftc.teamcode.powerplayV2.ClawSubsystem;
import org.firstinspires.ftc.teamcode.powerplayV2.FrontSliderSubsystem;

public class AutoConeCollection extends CommandBase {
    private ClawSubsystem claw;
    private ArmSubsystem arm;
    private FrontSliderSubsystem front_slider;

    private SequentialCommandGroup actions;

    public AutoConeCollection(ClawSubsystem claw, ArmSubsystem arm, FrontSliderSubsystem front_slider) {
        this.claw = claw;
        this.arm = arm;
        this.front_slider = front_slider;

        actions = new SequentialCommandGroup(
                new InstantCommand(claw::grab, claw),
                new ParallelCommandGroup(
                        new InstantCommand(arm::setMid, arm),
                        new InstantCommand(front_slider::close, front_slider)
                )
        );

        addRequirements(claw, arm, front_slider);
    }

    @Override
    public void initialize() {
        actions.schedule();
    }

    @Override
    public boolean isFinished() {
        return actions.isFinished();
    }
}
