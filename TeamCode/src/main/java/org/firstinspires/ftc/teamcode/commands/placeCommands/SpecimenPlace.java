package org.firstinspires.ftc.teamcode.commands.placeCommands;

import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.eStates.half;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pStates.midpoint;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pStates.specimenPlace;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pStates.specimenScore;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.BUMPER_RIGHT;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.BUTTON_DOWN;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.globalCommands.IdleFromIntake;
import org.firstinspires.ftc.teamcode.commands.intakeCommands.Pickup;
import org.firstinspires.ftc.teamcode.subsystems.Arm.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Arm.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Gripper.GripperSubsystem;
import org.firstinspires.ftc.teamcode.utils.BT.BTController;

public class SpecimenPlace extends SequentialCommandGroup{
    public SpecimenPlace(ExtensionSubsystem extension, PivotSubsystem pivot, ChassisSubsystem chassis, GripperSubsystem gripper, BTController controller){
        super(
                new ParallelCommandGroup(
                    pivot.setWithProfile(specimenScore,80,300),
                    extension.setExtension(-0.3),
                    gripper.setPickup()
                ),
                new WaitUntilCommand(controller.m_buttonsSuppliers[BUTTON_DOWN.ordinal()]),
                gripper.setMid(),
                pivot.setWithProfile(specimenPlace,30,150),
                new WaitUntilCommand(controller.m_buttonsSuppliers[BUTTON_DOWN.ordinal()]),
                gripper.openClaw(),
                new WaitCommand(200),
                new IdleFromIntake(extension, pivot, chassis, gripper)


        );
    }
}
