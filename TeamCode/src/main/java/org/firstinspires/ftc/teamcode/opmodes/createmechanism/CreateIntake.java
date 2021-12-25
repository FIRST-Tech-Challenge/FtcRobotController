package org.firstinspires.ftc.teamcode.opmodes.createmechanism;

import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
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
    private GamepadEx op;

    private MoveIntakeWithTrigger seGrabber;
    private MoveIntakeWithTrigger seReleaser;
    private StopIntake stopIntake;

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

    public CreateIntake(final HardwareMap hwMap, final String deviceName, Telemetry telemetry){
        this.deviceName = deviceName;
        this.hwMap = hwMap;
        this.telemetry = telemetry;

    }

    public void create(){
        telemetry.addData("create intake","creating");
        intake = new IntakeSubsystem(hwMap,deviceName);


        Trigger rTrigger = new Trigger(() -> op.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5);
        Trigger lTrigger = new Trigger(() -> op.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5);

        seGrabber = createMoveIntake(RIGHT_TRIGGER_POWER);
        seReleaser = createMoveIntake(LEFT_TRIGGER_POWER);

        rTrigger.whileActiveContinuous(seGrabber);
        lTrigger.whileActiveContinuous(seReleaser);

        stopIntake = createStopIntake();

        intake.setDefaultCommand(new PerpetualCommand(stopIntake));
    }

    public void createAuto(){
        telemetry.addData("create intake","creating");
        intake = new IntakeSubsystem(hwMap,deviceName);

        seGrabber = createMoveIntake(RIGHT_TRIGGER_POWER);
        seReleaser = createMoveIntake(LEFT_TRIGGER_POWER);

        stopIntake = createStopIntake();

        intake.setDefaultCommand(new PerpetualCommand(stopIntake));
    }

    private MoveIntakeWithTrigger createMoveIntake(double power){
        return new MoveIntakeWithTrigger(intake, power, telemetry);

    }

    private StopIntake createStopIntake(){
        return new StopIntake(intake, telemetry);
    }

    public MoveIntakeWithTrigger getSeGrabber(){
        return seGrabber;
    }

    public MoveIntakeWithTrigger getSeReleaser(){
        return seReleaser;
    }

    public StopIntake getStopIntake(){
        return stopIntake;
    }
}
