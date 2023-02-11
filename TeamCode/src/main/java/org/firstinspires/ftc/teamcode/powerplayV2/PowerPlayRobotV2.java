package org.firstinspires.ftc.teamcode.powerplayV2;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.teamcode.freightfrenzy.BucketCommand;
import org.firstinspires.ftc.teamcode.powerplayV2.commands.ConeCollection;
import org.firstinspires.ftc.teamcode.powerplayV2.commands.ResetSliderBasket;
import org.firstinspires.ftc.teamcode.powerplayV2.commands.SlidersGroup;
import org.firstinspires.ftc.teamcode.robotbase.RobotEx;

public class PowerPlayRobotV2 extends RobotEx {
    private ClawSubsystem claw;
    private ElevatorSubsystem elevator;
    private FrontSliderSubsystem frontSlider;
//    private LimitSwitchSubsystem rightServoLim, leftServoLim;
    private ArmSubsystem arm;
    private BasketSubsystem basket;
//    private ConeDetectorSubsystem cone_detector;
    private RumbleCommand rumbleCommand;

    private int index = 0;

    public PowerPlayRobotV2(HardwareMap hardwareMap, Telemetry telemetry, Gamepad driverOp,
                            Gamepad toolOp) {
        super(hardwareMap, telemetry, driverOp, toolOp, false, true,
                true, true, true, true,
                true);
    }

    @Override
    public void initMechanisms(HardwareMap hardwareMap) {
        claw = new ClawSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap);
        frontSlider = new FrontSliderSubsystem(hardwareMap);
        arm = new ArmSubsystem(hardwareMap);
        basket = new BasketSubsystem(hardwareMap);
//        cone_detector = new ConeDetectorSubsystem(hardwareMap, 20);

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
        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(new RumbleCommand(toolOp));

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

//        new Trigger(() -> -toolOp.getLeftY() >= 0.6).whenActive(new ConditionalCommand(
//                new InstantCommand(frontSlider::open, frontSlider),
//                new InstantCommand(frontSlider::stop, frontSlider),
//                () -> rightServoLim.getState() && leftServoLim.getState()));
//        new Trigger(() -> -toolOp.getLeftY() >= 0.6).whenActive(new ConditionalCommand(
//                new InstantCommand(frontSlider::open, frontSlider),
//                new InstantCommand(frontSlider::stop, frontSlider),
//                () -> rightServoLim.getState() && leftServoLim.getState()));


//
//        //Limit(Bound) Switches
//        new Trigger(() -> rightServoLim.getState()).whenActive(
//                new InstantCommand(frontSlider::stopRight, frontSlider)
//        );
//
//        new Trigger(() -> leftServoLim.getState()).whenActive(
//                new InstantCommand(frontSlider::stopLeft, frontSlider)
//        );


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

        toolOp.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(new AutonomousCommandWOPassThrough(0, claw, elevator, basket,
                        arm, telemetryEx));



        //////////////////////////////////----- Auto Actions -----//////////////////////////////////
//        toolOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//                        .whenPressed(new SequentialCommandGroup(
//                                new FrontSliderConeCommand(frontSlider,
//                                        cone_detector::isConeDetected, arm),
//                                new RumbleCommand(toolOp)
//                        ));
//
//        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//                .whenPressed(
//                        new SequentialCommandGroup(
//                                new ParallelCommandGroup(
//                                        new ConeCollection(claw, arm, frontSlider),
//                                        new ResetSliderBasket(slider, basket) // Reset Slider and Basket to Original Pos
//                                ),
//                                new RumbleCommand(driverOp),
//                                new InstantCommand(arm::setTravel, arm), //Transfer the cone
//                                new InstantCommand(claw::release, claw) // Release the cone to tha basket
//                        )
//                );
//
//        new Trigger(() -> toolOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.8)
//                .whenActive(new SequentialCommandGroup(
//                        new InstantCommand(basket::setOuttake, basket), //Outtake Cone
//                        new WaitCommand(1000), //Wait till the cone drops
//                        new ParallelCommandGroup( // Rumble, Reset Slider and Basket to Original Pos
//                                new RumbleCommand(driverOp),
//                                new ResetSliderBasket(slider, basket)
//                        )
//                ));
    }
}