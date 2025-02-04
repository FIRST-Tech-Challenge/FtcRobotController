package org.firstinspires.ftc.teamcode.commands.intakeCommands;

import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.eStates.half;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.eStates.quarter;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pStates.midpoint;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pStates.wingPlace;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.BUMPER_RIGHT;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.BUTTON_DOWN;

import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.globalCommands.IdleFromIntake;
import org.firstinspires.ftc.teamcode.subsystems.Arm.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Arm.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Gripper.GripperSubsystem;
import org.firstinspires.ftc.teamcode.utils.BT.BTController;

public class SpecimenPickup extends SequentialCommandGroup{
    public SpecimenPickup(ExtensionSubsystem extension, PivotSubsystem pivot, ChassisSubsystem chassis, GripperSubsystem gripper, BTController controller){
        super(
                pivot.setWithProfile(wingPlace,80,300),
                (gripper.setMid()),
                (gripper.openClaw()),
                (extension.setExtension(quarter)),
                (chassis.slowDriving(0.4)),
                new WaitUntilCommand(controller.m_buttonsSuppliers[BUTTON_DOWN.ordinal()]),
                gripper.openClaw(),
                new WaitUntilCommand(controller.m_buttonsSuppliers[BUTTON_DOWN.ordinal()]),
                pivot.setWithProfile(140,80,300),
                new IdleFromIntake(extension,pivot,chassis,gripper)

        );
    }
}
