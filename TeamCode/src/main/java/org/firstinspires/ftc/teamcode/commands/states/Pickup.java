package org.firstinspires.ftc.teamcode.commands.states;

import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.eStates.closed;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pStates.idle;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pStates.pickup;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pStates.up;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.BUMPER_RIGHT;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.BUTTON_DOWN;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Arm.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Gripper.GripperSubsystem;
import org.firstinspires.ftc.teamcode.utils.BT.BTController;

public class Pickup extends SequentialCommandGroup{
    public Pickup(ExtensionSubsystem extension, PivotSubsystem pivot, ChassisSubsystem chassis, GripperSubsystem gripper, BTController controller){
        super(
                new WaitUntilCommand(controller.m_buttonsSuppliers[BUMPER_RIGHT.ordinal()]),
                gripper.openClaw(),
                new WaitCommand(200),
                pivot.set(pickup),
                new WaitUntilCommand(controller.m_buttonsSuppliers[BUMPER_RIGHT.ordinal()]),
                gripper.closeClaw(),
                new WaitUntilCommand(controller.m_buttonsSuppliers[BUMPER_RIGHT.ordinal()]),
                pivot.set(up),
                new WaitUntilCommand(controller.m_buttonsSuppliers[BUMPER_RIGHT.ordinal()])
        );

    }
}
