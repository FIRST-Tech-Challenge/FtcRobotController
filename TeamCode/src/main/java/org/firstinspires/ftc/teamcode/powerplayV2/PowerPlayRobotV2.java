package org.firstinspires.ftc.teamcode.powerplayV2;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.powerplayV2.commands.BasketCommand;
import org.firstinspires.ftc.teamcode.powerplayV2.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.powerplayV2.commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.powerplayV2.commands.ElevatorManualCommand;
import org.firstinspires.ftc.teamcode.powerplayV2.commands.FrontSliderConeCommand;
import org.firstinspires.ftc.teamcode.powerplayV2.commands.FrontSliderRetractCommand;
import org.firstinspires.ftc.teamcode.powerplayV2.commands.ResetSliderBasket;
import org.firstinspires.ftc.teamcode.powerplayV2.commands.RumbleCommand;
import org.firstinspires.ftc.teamcode.powerplayV2.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.powerplayV2.subsystems.BasketSubsystem;
import org.firstinspires.ftc.teamcode.powerplayV2.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.powerplayV2.subsystems.ConeDetectorSubsystem;
import org.firstinspires.ftc.teamcode.powerplayV2.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.powerplayV2.subsystems.FrontSliderSubsystem;
import org.firstinspires.ftc.teamcode.powerplayV2.subsystems.LimitSwitchSubsystem;
import org.firstinspires.ftc.teamcode.robotbase.GamepadExEx;
import org.firstinspires.ftc.teamcode.robotbase.RobotEx;

public class PowerPlayRobotV2 extends RobotEx {
    private ClawSubsystem claw;
    private ElevatorSubsystem elevator;
    private FrontSliderSubsystem frontSlider;
    private LimitSwitchSubsystem rightServoLim, leftServoLim;
    private ArmSubsystem arm;
    private BasketSubsystem basket;
    private ConeDetectorSubsystem cone_detector;
    private RumbleCommand rumbleCommand;

    private int index = 0;

    public PowerPlayRobotV2(HardwareMap hardwareMap, Telemetry telemetry, GamepadExEx driverOp,
                            GamepadExEx toolOp) {
        super(hardwareMap, telemetry, driverOp, toolOp, OpModeType.TELEOP, true,
                false, true, true, true,
                true);
    }

    @Override
    public void initMechanismsAutonomous(HardwareMap hardwareMap) {
        claw = new ClawSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);
        rightServoLim = new LimitSwitchSubsystem(hardwareMap, "rightSwitch");
        leftServoLim = new LimitSwitchSubsystem(hardwareMap, "leftSwitch");
        frontSlider = new FrontSliderSubsystem(hardwareMap, () -> rightServoLim.getState(),
                () -> leftServoLim.getState());
        arm = new ArmSubsystem(hardwareMap, telemetry);
        basket = new BasketSubsystem(hardwareMap);
        cone_detector = new ConeDetectorSubsystem(hardwareMap, 15);

        //        PerpetualCommand autoLoop = new PerpetualCommand(new SequentialCommandGroup(
//                new SlidersGroup(slider, frontSlider, true),
//                new InstantCommand(basket::setOuttake, basket), //Outtake the Cone
//                new WaitCommand(1000),
//                new ParallelCommandGroup(
//                        new ConeCollection(claw, arm, frontSlider), // Collect Cone
//                        new ResetSliderBasket(slider, basket) // Reset Balancers
//                ),
//                new InstantCommand(arm::setTravel, arm), //Transfer the cone
//                new InstantCommand(claw::release, claw), // Release the cone to tha basket
//                new InstantCommand(arm::setMid, arm) // Set Arm in Mid position
//        ));

//        autoLoop.interruptOn(
//                () -> time >= 25;
//        );
    }

    @Override
    public void initMechanismsTeleOp(HardwareMap hardwareMap) {
        claw = new ClawSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);
        rightServoLim = new LimitSwitchSubsystem(hardwareMap, "rightSwitch");
        leftServoLim = new LimitSwitchSubsystem(hardwareMap, "leftSwitch");
        frontSlider = new FrontSliderSubsystem(hardwareMap, () -> rightServoLim.getState(),
                () -> leftServoLim.getState());
        arm = new ArmSubsystem(hardwareMap, telemetry);
        basket = new BasketSubsystem(hardwareMap);
        cone_detector = new ConeDetectorSubsystem(hardwareMap, 15);

        //Autonomous Commands
//        PerpetualCommand autoLoop = new PerpetualCommand(new SequentialCommandGroup(
//                new SlidersGroup(slider, frontSlider, true),
//                new InstantCommand(basket::setOuttake, basket), //Outtake the Cone
//                new WaitCommand(1000),
//                new ParallelCommandGroup(
//                        new ConeCollection(claw, arm, frontSlider), // Collect Cone
//                        new ResetSliderBasket(slider, basket) // Reset Balancers
//                ),
//                new InstantCommand(arm::setTravel, arm), //Transfer the cone
//                new InstantCommand(claw::release, claw), // Release the cone to tha basket
//                new InstantCommand(arm::setMid, arm) // Set Arm in Mid position
//        ));

//        autoLoop.interruptOn(
//                () -> time >= 25;
//        );

        /////////////////////////////////----- Manual Actions -----/////////////////////////////////

        /////////////////////////////////////////// Claw ///////////////////////////////////////////
        toolOp.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ClawCommand(claw));

        ////////////////////////////////////////// Slider //////////////////////////////////////////

        //Semi Manual Levels
        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new ElevatorCommand(elevator, ElevatorSubsystem.Level.LOW));
        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new ElevatorCommand(elevator, ElevatorSubsystem.Level.MID));
        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(new ElevatorCommand(elevator, ElevatorSubsystem.Level.HIGH));

        //Manual Height Adjustment
        new Trigger(() -> toolOp.getLeftY() >= 0.6).whileActiveContinuous(
                new ElevatorManualCommand(elevator, 1));
        new Trigger(() -> toolOp.getLeftY() <= -0.6).whileActiveContinuous(
                new ElevatorManualCommand(elevator, -1));

        //Only for rumble testing
//        toolOp.getGamepadButton(GamepadKeys.Button.BACK)
//                .whenPressed(new RumbleCommand(toolOp));

        /////////////////////////////////////// Front Slider ///////////////////////////////////////

//        Manual Length
//        new Trigger(() -> -toolOp.getLeftY() >= 0.6).toggleWhenActive(
//                new InstantCommand(frontSlider::open, frontSlider),
//                new InstantCommand(frontSlider::stop, frontSlider));

        new Trigger(() -> -toolOp.getRightY() >= 0.6).whenActive(
                new InstantCommand(frontSlider::open, frontSlider)
        );
        new Trigger(() -> -toolOp.getRightY() < 0.6 && -toolOp.getRightY() >= 0).whenActive(
                new InstantCommand(frontSlider::stop, frontSlider)
        );

        new Trigger(() -> -toolOp.getRightY() <= -0.6).whenActive(
                new InstantCommand(frontSlider::close, frontSlider)
        );
        new Trigger(() -> -toolOp.getRightY() > -0.6 && -toolOp.getRightY() <= 0).whenActive(
                new InstantCommand(frontSlider::stop, frontSlider)
        );

        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new FrontSliderRetractCommand(frontSlider));

        //Limit(Bound) Switches
        new Trigger(() -> rightServoLim.getState()).whenActive(
                new InstantCommand(frontSlider::stopRight, frontSlider)
        );

        new Trigger(() -> leftServoLim.getState()).whenActive(
                new InstantCommand(frontSlider::stopLeft, frontSlider)
        );


        /////////////////////////////////////////// Arm ////////////////////////////////////////////

        toolOp.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(arm::toggleState, arm));

        toolOp.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(arm::setMid, arm));

        new Trigger(() -> toolOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.5)
                .whileActiveContinuous(new InstantCommand(arm::increasePos));
        new Trigger(() -> toolOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.5)
                .whileActiveContinuous(new InstantCommand(arm::decreasePos));


        ////////////////////////////////////////// Basket //////////////////////////////////////////

        toolOp.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new BasketCommand(basket));

        ////////////////////////////// Autonomous Test w/o Pas Through /////////////////////////////

//        toolOp.getGamepadButton(GamepadKeys.Button.BACK)
//                .whenPressed(new AutonomousCommandWOPassThrough(0, claw, elevator, basket,
//                        arm, telemetryEx));



        //////////////////////////////////----- Auto Actions -----//////////////////////////////////
        new Trigger(() -> cone_detector.isConeDetected())
                .whenActive(toolOp::rumble);

        toolOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                        .whenPressed(new SequentialCommandGroup(
                                new FrontSliderConeCommand(frontSlider,
                                        cone_detector::isConeDetected, arm),
                                new RumbleCommand(toolOp)
                        ));

        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new InstantCommand(claw::grab, claw),
                                                new WaitCommand(500),
                                                new ParallelCommandGroup(
                                                        new InstantCommand(arm::setMid, arm),
                                                        new FrontSliderRetractCommand(frontSlider)
                                                )
                                        ),
                                        new ElevatorCommand(elevator, ElevatorSubsystem.Level.LOW),
                                        new InstantCommand(basket::setTravel, basket) // Reset Slider and Basket to Original Pos
                                ),
                                new WaitCommand(600),
                                new InstantCommand(arm::setTravel, arm),
                                new WaitCommand(200),
                                new RumbleCommand(driverOp),
                                new WaitCommand(100),
                                new InstantCommand(claw::release, claw), // Release the cone to tha basket
                                new WaitCommand(400),
                                new InstantCommand(arm::setMid, arm)
                        )
                );

        new Trigger(() -> toolOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.8)
                .whenActive(new SequentialCommandGroup(
                        new InstantCommand(basket::setOuttake, basket), //Outtake Cone
                        new WaitCommand(1500), //Wait till the cone drops
                        new ParallelCommandGroup( // Rumble, Reset Slider and Basket to Original Pos
                                new RumbleCommand(driverOp),
                                new ResetSliderBasket(elevator, basket)
                        )
                ));
    }
}