package org.firstinspires.ftc.teamcode.powerplay;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotbase.RobotEx;

public class PowerPlayRobot extends RobotEx {
    private ClawSubsystem claw;
    private SliderSubsystem slider;
    private ArmSubsystem arm;
    private ConeDetectorSubsystem cone_detector;

    public PowerPlayRobot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad driverOp,
                          Gamepad toolOp) {
        super(hardwareMap, telemetry, driverOp, toolOp, false, true,
                true, true, true, true,
                true);
    }

    @Override
    public void initMechanisms(HardwareMap hardwareMap) {
        ////////////////////////////////////////// Claw //////////////////////////////////////////
//        claw = new ClawSubsystem(hardwareMap);
//        toolOp.getGamepadButton(GamepadKeys.Button.A)
//                .whenPressed(new ClawCommand(claw));

        ////////////////////////////////////////// Slider //////////////////////////////////////////
//        slider = new SliderSubsystem(hardwareMap);
//
//        //Levels
//        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//
//                .whenPressed(new SliderCommand(slider, SliderSubsystem.Level.LOW));
//        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
//                .whenPressed(new SliderCommand(slider, SliderSubsystem.Level.MID));
//        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//                .whenPressed(new SliderCommand(slider, SliderSubsystem.Level.HIGH));
//        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
//                .whenPressed(new SliderCommand(slider, SliderSubsystem.Level.PARK));
//        toolOp.getGamepadButton(GamepadKeys.Button. LEFT_STICK_BUTTON)
//                .whenPressed(new SliderCommand(slider, SliderSubsystem.Level.GRAB));
//
////        Manual Height Adjustment
//        new Trigger(() -> toolOp.getLeftY() >= 0.8).whileActiveContinuous(
//                new SliderManualCommand(slider, 1));
//        new Trigger(() -> toolOp.getLeftY() <= -0.8).whileActiveContinuous(
//                new SliderManualCommand(slider, -1));

        /////////////////////////////////////////// Arm ////////////////////////////////////////////
//        arm = new ArmSubsystem(hardwareMap);
//
//        new Trigger(() -> toolOp.getRightY() >= 0.8).whenActive(
//                new InstantCommand(arm::setForward, arm));
//        new Trigger(() -> toolOp.getRightY() <= -0.8).whenActive(
//                new InstantCommand(arm::setBackward, arm));

        /////////////////////////////////////// Auto Actions ///////////////////////////////////////
//        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//                .whenPressed(
//                        new SequentialCommandGroup(
//                                new InstantCommand(arm::setBackward, arm),
//                                new ParallelCommandGroup(
//                                        new SliderCommand(slider, SliderSubsystem.Level.GRAB),
//                                        new InstantCommand(claw::release, claw)
//                                )
//                        )
//                );
//
//        toolOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//                .whenPressed(
//                        new SequentialCommandGroup(
//                                new InstantCommand(arm::setForward, arm),
//                                new ParallelCommandGroup(
//                                        new SliderCommand(slider, SliderSubsystem.Level.GRAB),
//                                        new InstantCommand(claw::release, claw)
//                                )
//                        )
//                );

        ////////////////////////////////////////// Cone Detector //////////////////////////////////////////
//        cone_detector = new ConeDetectorSubsystem(hardwareMap, 30);
//
//        toolOp.getGamepadButton(GamepadKeys.Button.START)
//                .whenPressed(new InstantCommand(cone_detector::toogleState, cone_detector));
//
//        if (cone_detector.isEnabled()) {
//            new Trigger(cone_detector::isConeDetected)
//                    .whenActive(new SequentialCommandGroup(
//                            new InstantCommand(claw::grab, claw),
//                            new SliderCommand(slider, SliderSubsystem.Level.PARK),
//                            new InstantCommand(arm::toggleState, arm)
//                    ));
//        }
    }
}