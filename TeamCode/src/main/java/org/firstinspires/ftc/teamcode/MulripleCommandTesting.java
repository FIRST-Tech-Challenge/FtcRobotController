package org.firstinspires.ftc.teamcode;

import android.text.style.IconMarginSpan;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.powerplayV2.ArmAutoCommand;
import org.firstinspires.ftc.teamcode.powerplayV2.ArmSubsystem;
import org.firstinspires.ftc.teamcode.powerplayV2.BasketSubsystem;
import org.firstinspires.ftc.teamcode.powerplayV2.ClawSubsystem;
import org.firstinspires.ftc.teamcode.powerplayV2.ElevatorCommand;
import org.firstinspires.ftc.teamcode.powerplayV2.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.powerplayV2.commands.ResetSliderBasket;

@TeleOp
public class MulripleCommandTesting extends CommandOpMode {
    SequentialCommandGroup actions;
    ClawSubsystem claw;
    ElevatorSubsystem elevator;
    BasketSubsystem basket;
    ArmSubsystem arm;

    @Override
    public void initialize() {
        claw = new ClawSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);
        basket = new BasketSubsystem(hardwareMap);
        arm = new ArmSubsystem(hardwareMap);

        actions = new SequentialCommandGroup(
                new InstantCommand(claw::grab, claw),
                new WaitCommand(600),
                new InstantCommand(arm::setTravel),
                new WaitCommand(600),
                new InstantCommand(claw::release, claw),
                new WaitCommand(700),
                new InstantCommand(arm::setMid, arm),
                new WaitCommand(100),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new ElevatorCommand(elevator, ElevatorSubsystem.Level.HIGH),
                                new InstantCommand(basket::setOuttake, basket), //Outtake the Cone
                                new WaitCommand(1500),
                                new ResetSliderBasket(elevator, basket)
                        ),
                        new InstantCommand(arm::setIntake, arm)
//                        new ArmAutoCommand(arm, () -> index)
                ),
                new WaitCommand(1000),
                new InstantCommand(arm::setMid, arm)
        );
    }

    @Override
    public void run() {
        actions.schedule();

        while(!actions.isFinished());

        actions.schedule();
    }
}
