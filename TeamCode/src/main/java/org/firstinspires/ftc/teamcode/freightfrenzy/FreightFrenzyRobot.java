package org.firstinspires.ftc.teamcode.freightfrenzy;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.InstanceId;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotbase.RobotEx;

public class FreightFrenzyRobot extends RobotEx {
    private BucketSubsystem bucket;
    private CarouselSubsystem carousel;
    private SliderSubsystem slider;
    private IntakeSubsystem intake;

    public FreightFrenzyRobot(HardwareMap hardwareMap, Telemetry telemetry, GamepadEx driverOp,
                              GamepadEx toolOp) {
        super(hardwareMap, telemetry, driverOp, toolOp);
    }

    @Override
    public void initMechanisms(HardwareMap hardwareMap) {
        ////////////////////////////////////////// Bucket //////////////////////////////////////////
        bucket = new BucketSubsystem(hardwareMap);
        toolOp.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new BucketCommand(bucket));

        ///////////////////////////////////////// Carousel /////////////////////////////////////////
        carousel = new CarouselSubsystem(hardwareMap);
        new Trigger(() -> toolOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.5)
                .whileActiveContinuous(new InstantCommand(carousel::rotateClockwise, carousel))
                .whenInactive(new InstantCommand(carousel::stop, carousel));
        new Trigger(() -> toolOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.5)
                .whileActiveContinuous(new InstantCommand(carousel::rotateCounterClockwise, carousel))
                .whenInactive(new InstantCommand(carousel::stop, carousel));

        //Carousel Proc
        toolOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(
                        new PerpetualCommand(new SequentialCommandGroup(
                                    new RunCommand(carousel::rotateClockwise, carousel),
                                    new WaitCommand(8000),
                                    new RunCommand(carousel::stop, carousel),
                                    new WaitCommand(1000)
                        )),
                        new InstantCommand(carousel::stop, carousel)
                );

        ////////////////////////////////////////// Slider //////////////////////////////////////////
        slider = new SliderSubsystem(hardwareMap);

        //Levels
        toolOp.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new SliderCommand(slider, SliderSubsystem.Level.ONE));
        toolOp.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new SliderCommand(slider, SliderSubsystem.Level.TWO));
        toolOp.getGamepadButton(GamepadKeys.Button.B)
                .whileHeld(new SliderCommand(slider, SliderSubsystem.Level.THREE));

        //Manual Height Adjustment
        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whileHeld(new SliderManualCommand(slider, -1));
        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whileHeld(new SliderManualCommand(slider, 1));

        ////////////////////////////////////////// Intake //////////////////////////////////////////
        intake = new IntakeSubsystem(hardwareMap);
        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(
                        new ParallelCommandGroup(
                                new InstantCommand(intake::forward, intake),
                                new SliderCommand(slider, SliderSubsystem.Level.INTAKE),
                                new InstantCommand(bucket::intake, bucket)
//                                new PerpetualCommand(new InstantCommand(intake::run, intake))),
                        ),
                        new ParallelCommandGroup(
                                new InstantCommand(bucket::rest, bucket),
                                new SliderCommand(slider, SliderSubsystem.Level.PARK),
                                new SequentialCommandGroup(
                                        new InstantCommand(intake::reverse, intake),
                                        new WaitCommand(500),
                                        new InstantCommand(intake::stop, intake)
                                )
                        )
                );

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(
                        new InstantCommand(intake::reverse, slider)
                ).whenReleased(
                        new InstantCommand(intake::stop, slider)
                );

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(
                        new InstantCommand(intake::forward, slider)
                ).whenReleased(
                        new InstantCommand(intake::stop, slider)
                );
    }
}