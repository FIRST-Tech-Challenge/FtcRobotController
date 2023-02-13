//package org.firstinspires.ftc.teamcode.powerplayV2.commands;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.teamcode.powerplayV2.subsystems.ArmSubsystem;
//import org.firstinspires.ftc.teamcode.powerplayV2.subsystems.BasketSubsystem;
//import org.firstinspires.ftc.teamcode.powerplayV2.subsystems.ClawSubsystem;
//import org.firstinspires.ftc.teamcode.powerplayV2.subsystems.ElevatorSubsystem;
//import org.firstinspires.ftc.teamcode.powerplayV2.subsystems.FrontSliderSubsystem;
//
//public class AutonomousCommand extends CommandBase {
//    // Constants
//    ElevatorSubsystem.Level LEVEL = ElevatorSubsystem.Level.HIGH;
//    HardwareMap hardwareMap;
//
//    //Subsystems
//    ClawSubsystem claw;
//    ElevatorSubsystem elevator;
//    BasketSubsystem basket;
//    ArmSubsystem arm;
//    FrontSliderSubsystem frontSlider;
//
//    SequentialCommandGroup actions;
//
//    public AutonomousCommand(HardwareMap hardwareMap, int index) {
//        this.hardwareMap = hardwareMap;
//
//        claw = new ClawSubsystem(hardwareMap);
//        elevator = new ElevatorSubsystem(hardwareMap);
//        basket = new BasketSubsystem(hardwareMap);
////        arm = new ArmSubsystem(hardwareMap);
////        frontSlider = new FrontSliderSubsystem(hardwareMap);
//
//        actions = new SequentialCommandGroup(
//                new ParallelCommandGroup(
//                        new SlidersGroup(elevator, frontSlider, true),// Extend Both Sliders
//                        new ArmAutoCommand(arm, () -> index)
//                ),
//                new InstantCommand(basket::setOuttake, basket), //Outtake the Cone
//                new WaitCommand(1000),
//                new ParallelCommandGroup(
//                        new ConeCollection(claw, arm, frontSlider), // Collect Cone
//                        new ResetSliderBasket(elevator, basket) // Reset Balancers
//                ),
//                new InstantCommand(arm::setTravel, arm), //Transfer the cone
//                new InstantCommand(claw::release, claw), // Release the cone to tha basket
//                new InstantCommand(arm::setMid, arm) // Set Arm in Mid position
//        );
//
//        addRequirements(claw, elevator, basket, arm, frontSlider);
//    }
//
//    @Override
//    public void initialize() {
//        actions.schedule();
//    }
//
//    @Override
//    public boolean isFinished() {
//        return actions.isFinished();
//    }
//}
