package org.firstinspires.ftc.teamcode.Slidy_PPV2.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.Slidy_PPV2.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Slidy_PPV2.subsystems.FrontSliderSubsystem;

public class SlidersGroup extends CommandBase {
    private ElevatorSubsystem elevator;
    private FrontSliderSubsystem front_slider;

    private ParallelCommandGroup actions;

    public SlidersGroup(ElevatorSubsystem elevator, FrontSliderSubsystem front_slider, Boolean extend) {
        this.elevator = elevator;
        this.front_slider = front_slider;

        actions = extend ? new ParallelCommandGroup(
                new ElevatorCommand(elevator, ElevatorSubsystem.Level.HIGH),
                new InstantCommand(() -> front_slider.open(0.3))
        ) : new ParallelCommandGroup(
                new ElevatorCommand(elevator, ElevatorSubsystem.Level.LOW),
                new InstantCommand(front_slider::close)
        );

        addRequirements(elevator, front_slider);
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
