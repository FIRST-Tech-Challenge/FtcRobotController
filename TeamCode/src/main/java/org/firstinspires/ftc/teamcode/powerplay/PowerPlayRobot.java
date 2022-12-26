package org.firstinspires.ftc.teamcode.powerplay;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotbase.RobotEx;

public class PowerPlayRobot extends RobotEx {
    private ClawSubsystem claw;
    private SliderSubsystem slider;
    private ArmSubsystem arm;
    private ConeDetectorSubsystem cone_detector;

    public PowerPlayRobot(HardwareMap hardwareMap, Telemetry telemetry, GamepadEx driverOp,
                          GamepadEx toolOp) {
        super(hardwareMap, telemetry, driverOp, toolOp, false, true,
                false, false, false, false,
                true);
    }

    @Override
    public void initMechanisms(HardwareMap hardwareMap) {
        ////////////////////////////////////////// Claw //////////////////////////////////////////
        claw = new ClawSubsystem(hardwareMap);
        toolOp.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ClawCommand(claw));

        ////////////////////////////////////////// Slider //////////////////////////////////////////
//        slider = new SliderSubsystem(hardwareMap);
//
//        //Levels
//        toolOp.getGamepadButton(GamepadKeys.Button.X)
//                .whenPressed(new SliderCommand(slider, SliderSubsystem.Level.ONE));
//        toolOp.getGamepadButton(GamepadKeys.Button.Y)
//                .whenPressed(new SliderCommand(slider, SliderSubsystem.Level.TWO));
//        toolOp.getGamepadButton(GamepadKeys.Button.B)
//                .whileHeld(new SliderCommand(slider, SliderSubsystem.Level.THREE));

        //Manual Height Adjustment
//        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//                .whileHeld(new SliderManualCommand(slider, -1));
//        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//                .whileHeld(new SliderManualCommand(slider, 1));

        ////////////////////////////////////////// Auto Actions //////////////////////////////////////////
        arm = new ArmSubsystem(hardwareMap);

        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        new SequentialCommandGroup(
                                new InstantCommand(arm::setBackward, arm),
                                new ParallelCommandGroup(
                                        new SliderCommand(slider, SliderSubsystem.Level.GRAB),
                                        new InstantCommand(claw::release, claw)
                                )
                        )
                );

        toolOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(
                        new SequentialCommandGroup(
                                new InstantCommand(arm::setForward, arm),
                                new ParallelCommandGroup(
                                        new SliderCommand(slider, SliderSubsystem.Level.GRAB),
                                        new InstantCommand(claw::release, claw)
                                )
                        )
                );

        ////////////////////////////////////////// Cone Detector //////////////////////////////////////////
        cone_detector = new ConeDetectorSubsystem(hardwareMap, 30);

        new Trigger(cone_detector::isConeDetected)
                .whenActive(new SequentialCommandGroup(
                        new ClawCommand(claw),
                        new SliderCommand(slider, SliderSubsystem.Level.PARK),
                        new InstantCommand(arm::toggleState, arm)
                ));
    }
}