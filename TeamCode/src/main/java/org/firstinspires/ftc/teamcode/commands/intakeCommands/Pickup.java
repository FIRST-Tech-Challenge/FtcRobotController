package org.firstinspires.ftc.teamcode.commands.intakeCommands;

import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pStates.pickup;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pStates.up;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.BUTTON_DOWN;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Arm.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Gripper.GripperSubsystem;
import org.firstinspires.ftc.teamcode.utils.BT.BTController;

public class Pickup extends SequentialCommandGroup{
    public Pickup(ExtensionSubsystem extension, PivotSubsystem pivot, ChassisSubsystem chassis, GripperSubsystem gripper, BTController controller){
        super(
                gripper.openClaw(),
                new WaitUntilCommand(controller.m_buttonsSuppliers[BUTTON_DOWN.ordinal()]),
                new WaitUntilCommand(()->!controller.m_buttonsSuppliers[BUTTON_DOWN.ordinal()].getAsBoolean()),
                pivot.setWithProfile(pickup,80,200),
                new WaitUntilCommand(controller.m_buttonsSuppliers[BUTTON_DOWN.ordinal()]),
                new WaitUntilCommand(()->!controller.m_buttonsSuppliers[BUTTON_DOWN.ordinal()].getAsBoolean()),
                gripper.closeClaw(),
                new WaitUntilCommand(controller.m_buttonsSuppliers[BUTTON_DOWN.ordinal()]),
                new WaitUntilCommand(()->!controller.m_buttonsSuppliers[BUTTON_DOWN.ordinal()].getAsBoolean()),
                pivot.setWithProfile(up,80,200),
                new WaitUntilCommand(controller.m_buttonsSuppliers[BUTTON_DOWN.ordinal()]),
                new WaitUntilCommand(()->!controller.m_buttonsSuppliers[BUTTON_DOWN.ordinal()].getAsBoolean())
        );

    }
}
