package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.command.SounderBotBaseRunCommand;
import org.firstinspires.ftc.teamcode.opmodes.OpModeTemplate;
import org.firstinspires.ftc.teamcode.opmodes.PowerMode;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DriveTrain;

/**
 * This is old mainTeleop
 */

@SuppressWarnings("unused")


@TeleOp (name = "SpecimenTest")
public class NewSpecificAttachmentTest extends OpModeTemplate {

    //private Servo specimenArmServo;
    SpecimenTest specimen;
    //private Servo specimenClawServo;
    private DriveTrain driveTrain;

    @Override
    public void initialize() {
        super.initialize();

        // Initialize the servos
        Servo specimenArmServo = hardwareMap.get(Servo.class,"specimenArm");
        Servo specimenClawServo = hardwareMap.get(Servo.class,"specimenClaw");
        specimen = new SpecimenTest(specimenArmServo,specimenClawServo,telemetry);
        //DriverFeedback feedback = new DriverFeedback(hardwareMap, driverGamepad, operatorGamepad, telemetry);
        driveTrain = new DriveTrain(hardwareMap, driverGamepad, telemetry, null, false, null);
        switchToMode(PowerMode.REGULAR);


        driverGamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenHeld(new InstantCommand(specimen::openClaw));

        driverGamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenHeld(new InstantCommand(specimen::closeClaw));

        /*
        driverGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenHeld(new InstantCommand(specimen::raiseArm));

        driverGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenHeld(new InstantCommand(specimen::dropArm));
         */

        driverGamepad.getGamepadButton((GamepadKeys.Button.DPAD_DOWN))
                        .whenHeld(new InstantCommand(specimen::pickSpecimen));

        driverGamepad.getGamepadButton((GamepadKeys.Button.DPAD_UP))
                .whenHeld(new InstantCommand(specimen::dropSpecimen));


        specimenClawServo.setPosition(0);
        specimenArmServo.setPosition(0);

        // Drivetrain speed
        // Drivetrain speed
        new Trigger(() -> gamepad1.right_trigger > 0.5)
                .whenActive(new InstantCommand(this::switchToNitroMode, driveTrain))
                .whenInactive(new InstantCommand(this::switchToRegularMode, driveTrain));

        new Trigger(() -> gamepad1.left_trigger > 0.5)
                .whenActive(new InstantCommand(this::switchToSlowMode, driveTrain))
                .whenInactive(new InstantCommand(this::switchToRegularMode, driveTrain));

        // Robot direction
        driverGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenHeld(new InstantCommand(driveTrain::ToggleDriveDirection, driveTrain));

        // servo test, for the speciman part claw thingy
        driverGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenHeld(new InstantCommand(driveTrain::ToggleDriveDirection, driveTrain));


        // Register all subsystems
        register(driveTrain);

        // update telemetry every loop
        schedule(SounderBotBaseRunCommand.createTelemetryEnabledOnlyInstance(telemetry));
    }

    @Override
    public void run() {
        super.run();
    }

    boolean fastMode = true;
    private void ToggleSpeed() {

        if(fastMode) {
            switchToMode(PowerMode.NITRO);
        } else {
            switchToMode(PowerMode.SLOW);
        }
    }

    private void switchToNitroMode() {
        switchToMode(PowerMode.NITRO);
    }

    private void switchToRegularMode() {
        switchToMode(PowerMode.REGULAR);
    }

    private void switchToSlowMode() {
        switchToMode(PowerMode.SLOW);
    }

    private void switchToMode(PowerMode powerMode) {
        driveTrain.setPowerRatio(powerMode.getPowerRatio());
    }
}
