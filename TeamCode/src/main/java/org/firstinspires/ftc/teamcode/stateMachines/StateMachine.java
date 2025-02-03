//package org.firstinspires.ftc.teamcode.stateMachines;
//
//import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.eStates.extended;
//import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pStates.score;
//import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pStates.scoreMidpoint;
//import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.BUMPER_RIGHT;
//import static org.firstinspires.ftc.teamcode.utils.BT.BTController.Buttons.BUTTON_DOWN;
//
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.RepeatCommand;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitUntilCommand;
//
//import org.firstinspires.ftc.teamcode.stateMachines.states.Idle;
//import org.firstinspires.ftc.teamcode.stateMachines.states.Intake;
//import org.firstinspires.ftc.teamcode.stateMachines.states.Pickup;
//import org.firstinspires.ftc.teamcode.stateMachines.states.Score;
//import org.firstinspires.ftc.teamcode.subsystems.Arm.ExtensionSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.Arm.PivotSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.Drivetrain.ChassisSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.Gripper.GripperSubsystem;
//import org.firstinspires.ftc.teamcode.utils.BT.BTController;
//
//public class StateMachine extends SequentialCommandGroup{
//    public StateMachine(ExtensionSubsystem extension, PivotSubsystem pivot, ChassisSubsystem chassis, GripperSubsystem gripper, BTController controller){
//        super(
//                new Idle(extension,pivot,chassis,gripper),
//                new WaitUntilCommand(controller.m_buttonsSuppliers[BUMPER_RIGHT.ordinal()]),
//                new Intake(extension,pivot,chassis,gripper),
//                new RepeatCommand(new Pickup(extension,pivot,chassis,gripper,controller))
//                        .interruptOn(controller.m_buttonsSuppliers[BUMPER_RIGHT.ordinal()]),
//                new Idle(extension,pivot,chassis,gripper),
//                new WaitUntilCommand(controller.m_buttonsSuppliers[BUMPER_RIGHT.ordinal()]),
//                new Score(extension,pivot,chassis,gripper),
//                new WaitUntilCommand(controller.m_buttonsSuppliers[BUMPER_RIGHT.ordinal()]),
//                gripper.openClaw(),
//                new WaitUntilCommand(controller.m_buttonsSuppliers[BUMPER_RIGHT.ordinal()])
//        );
//    }
//}
