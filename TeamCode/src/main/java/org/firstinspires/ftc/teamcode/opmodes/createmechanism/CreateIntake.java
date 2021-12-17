package org.firstinspires.ftc.teamcode.opmodes.createmechanism;

import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.carousel.MoveCarousel;
import org.firstinspires.ftc.teamcode.commands.carousel.StopCarousel;
import org.firstinspires.ftc.teamcode.commands.intake.MoveIntake;
import org.firstinspires.ftc.teamcode.commands.intake.MoveIntakeWithTrigger;
import org.firstinspires.ftc.teamcode.commands.intake.StopIntake;
import org.firstinspires.ftc.teamcode.subsystems.carousel.CarouselSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem;

public class CreateIntake {

    private IntakeSubsystem intake;
    private final HardwareMap hwMap;
    private final String deviceName;
    private final Telemetry telemetry;
    private final GamepadEx op;

    private static final double RIGHT_TRIGGER_POWER = -0.75;
    private static final double LEFT_TRIGGER_POWER = 0.6;

    public CreateIntake(final HardwareMap hwMap, final String deviceName, final GamepadEx op, Telemetry telemetry){
        this.deviceName = deviceName;
        this.hwMap = hwMap;
        this.op = op;
        this.telemetry = telemetry;

    }

    public CreateIntake(final HardwareMap hwMap, final String deviceName, final GamepadEx op, Telemetry telemetry, boolean autoCreate){
        this.deviceName = deviceName;
        this.hwMap = hwMap;
        this.op = op;
        this.telemetry = telemetry;

        if (autoCreate) create();

    }

    public void create(){
        telemetry.addData("create intake","creating");
        intake = new IntakeSubsystem(hwMap,deviceName);


        MoveIntake seGrabber = new MoveIntake(intake, RIGHT_TRIGGER_POWER,LEFT_TRIGGER_POWER, () -> op.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), () -> op.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), telemetry);


        //MoveIntakeWithTrigger seGrabber = new MoveIntakeWithTrigger(intake, RIGHT_TRIGGER_POWER, () -> op.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), telemetry);
        //MoveIntakeWithTrigger seReleaser = new MoveIntakeWithTrigger(intake, LEFT_TRIGGER_POWER, () -> op.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), telemetry);
        //MoveIntakeWithTrigger seReleaser = new MoveIntakeWithTrigger(intake, RIGHT_TRIGGER_POWER, () -> op.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), telemetry);

        StopIntake stopIntake = new StopIntake(intake, telemetry);



        seGrabber.schedule(true);
        //seReleaser.schedule(true);
        intake.setDefaultCommand(new PerpetualCommand(stopIntake));
    }
}
