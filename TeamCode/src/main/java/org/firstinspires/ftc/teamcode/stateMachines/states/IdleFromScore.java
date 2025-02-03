package org.firstinspires.ftc.teamcode.stateMachines.states;

import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.eStates.closed;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pStates.idle;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Arm.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Arm.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Gripper.GripperSubsystem;

public class IdleFromScore extends SequentialCommandGroup{
    public IdleFromScore(ExtensionSubsystem extension, PivotSubsystem pivot, ChassisSubsystem chassis, GripperSubsystem gripper){
        super(
                pivot.setWithProfile(idle,80,150),
                gripper.setScore(),
                extension.setExtension(closed),
                chassis.stopSlowDriving()
        );
    }
}
