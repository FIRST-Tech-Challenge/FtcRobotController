package org.firstinspires.ftc.teamcode.commands.placeCommands;

import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.eStates.extended;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pStates.score;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pStates.scoreMidpoint;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.BUMPER_LEFT;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.BUTTON_DOWN;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.globalCommands.IdleFromScore;
import org.firstinspires.ftc.teamcode.subsystems.Arm.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Arm.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Gripper.GripperSubsystem;
import org.firstinspires.ftc.teamcode.utils.BT.BTController;

public class SpecimenDelivery extends SequentialCommandGroup{
    public SpecimenDelivery(ExtensionSubsystem extension, PivotSubsystem pivot, ChassisSubsystem chassis, GripperSubsystem gripper, BTController controller){
        super(
                chassis.slowDriving(0.4),
                new ParallelCommandGroup(
                    extension.setExtension(extended),
                    pivot.setWithProfile(180,80,300), gripper.setScore()
                ),
                new WaitUntilCommand(controller.m_buttonsSuppliers[BUTTON_DOWN.ordinal()]),
                gripper.openClaw(),
                new WaitUntilCommand(controller.m_buttonsSuppliers[BUMPER_LEFT.ordinal()]),
                new IdleFromScore(extension,pivot,chassis,gripper)
        );
    }
}
