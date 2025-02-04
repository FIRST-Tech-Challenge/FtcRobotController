package org.firstinspires.ftc.teamcode.stateMachines.specimenStates;

import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.eStates.quarter;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pStates.specimenIntake;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pStates.specimenScore;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.BUMPER_LEFT;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.BUTTON_DOWN;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Arm.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Gripper.GripperSubsystem;
import org.firstinspires.ftc.teamcode.utils.BT.BTController;

public class Place extends SequentialCommandGroup{
    public Place(ExtensionSubsystem extension, PivotSubsystem pivot, ChassisSubsystem chassis, GripperSubsystem gripper, BTController controller){
        super(
                pivot.setWithProfile(specimenScore,30,80),
                gripper.setSpecimenScore(),
                new WaitUntilCommand(controller.m_buttonsSuppliers[BUTTON_DOWN.ordinal()]),
                gripper.setSpecimenPlace(),
                new WaitUntilCommand(controller.m_buttonsSuppliers[BUTTON_DOWN.ordinal()]),
                gripper.openClaw(),
                new WaitUntilCommand(controller.m_buttonsSuppliers[BUMPER_LEFT.ordinal()]),
                pivot.setWithProfile(specimenIntake,30,80),
                gripper.setSpecimenPlace(),//intake is same angle
                extension.setExtension(quarter),
                new WaitUntilCommand(controller.m_buttonsSuppliers[BUTTON_DOWN.ordinal()]),
                gripper.closeClaw(),
                new WaitUntilCommand(controller.m_buttonsSuppliers[BUMPER_LEFT.ordinal()])

        );
    }
}
