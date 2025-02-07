package org.firstinspires.ftc.teamcode.commands.intakeCommands;

import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.eStates.half;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pStates.midpoint;
import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.BUMPER_RIGHT;

import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.globalCommands.IdleFromIntake;
import org.firstinspires.ftc.teamcode.subsystems.Arm.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Arm.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Gripper.GripperSubsystem;
import org.firstinspires.ftc.teamcode.utils.BT.BTController;

public class SampleIntake extends SequentialCommandGroup{
    public SampleIntake(ExtensionSubsystem extension, PivotSubsystem pivot, ChassisSubsystem chassis, GripperSubsystem gripper, BTController controller){
        super(
                pivot.setWithProfile(midpoint,80,300),
                (gripper.setPickup()),
                (gripper.openClaw()),
                (extension.setExtension(half)),
                (chassis.slowDriving(0.4)),
                new RepeatCommand(new Pickup(extension,pivot,chassis,gripper,controller)).interruptOn(controller.m_buttonsSuppliers[BUMPER_RIGHT.ordinal()]),
                gripper.closeClaw(),
                new IdleFromIntake(extension,pivot,chassis,gripper)

        );
    }

}
