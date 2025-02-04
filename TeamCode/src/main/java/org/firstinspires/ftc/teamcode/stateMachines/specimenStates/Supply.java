package org.firstinspires.ftc.teamcode.stateMachines.specimenStates;

import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.eStates.extended;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pStates.wingPlace;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.BUMPER_RIGHT;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.BUTTON_DOWN;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.stateMachines.states.IdleFromScore;
import org.firstinspires.ftc.teamcode.stateMachines.states.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Arm.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Arm.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Gripper.GripperSubsystem;
import org.firstinspires.ftc.teamcode.utils.BT.BTController;

public class Supply extends SequentialCommandGroup{
    public Supply(ExtensionSubsystem extension, PivotSubsystem pivot, ChassisSubsystem chassis, GripperSubsystem gripper, BTController controller){
        super(
                new Intake(extension, pivot, chassis, gripper, controller),
                new WaitUntilCommand(controller.m_buttonsSuppliers[BUMPER_RIGHT.ordinal()]),
                pivot.setWithProfile(wingPlace,80,300),
                extension.setExtension(extended),
                gripper.setScore(),
                new WaitUntilCommand(controller.m_buttonsSuppliers[BUTTON_DOWN.ordinal()]),
                gripper.openClaw(),
                new WaitUntilCommand(controller.m_buttonsSuppliers[BUTTON_DOWN.ordinal()]),
                new IdleFromScore(extension, pivot, chassis, gripper),
                new WaitUntilCommand(controller.m_buttonsSuppliers[BUMPER_RIGHT.ordinal()])

        );
    }
}
