package org.firstinspires.ftc.teamcode.Slidy_PPV2.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Slidy_PPV2.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.Slidy_PPV2.subsystems.BasketSubsystem;
import org.firstinspires.ftc.teamcode.Slidy_PPV2.subsystems.ElevatorSubsystem;

public class SimpleAuto extends CommandBase {
    private ArmSubsystem arm;
    private BasketSubsystem basket;
    private ElevatorSubsystem elevator;

    private SequentialCommandGroup actions;

    public SimpleAuto(ArmSubsystem arm, BasketSubsystem basket, ElevatorSubsystem elevator) {
        this.arm = arm;
        this.basket = basket;
        this.elevator = elevator;

        actions = new SequentialCommandGroup(
                new InstantCommand(arm::setMid, arm),
                new WaitCommand(300),
                new ElevatorCommand(elevator, ElevatorSubsystem.Level.HIGH),
                new InstantCommand(basket::setOuttake, basket),
                new WaitCommand(1500),
                new ParallelCommandGroup(
                        new InstantCommand(basket::setTravel, basket),
                        new ElevatorCommand(elevator, ElevatorSubsystem.Level.LOW)
                )
        );
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
