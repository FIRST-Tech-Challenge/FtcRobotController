package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.command.SounderBotBaseRunCommand;
import org.firstinspires.ftc.teamcode.opmodes.OpModeTemplate;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.TeleFourWheelMecanumDriveTrain;

//@TeleOp (name = "test_teleop")
@SuppressWarnings("unused")
public class FourWheelMecanumDriveTeleop extends OpModeTemplate {

    private TeleFourWheelMecanumDriveTrain driveTrain;

    @Override
    public void initialize() {
        super.initialize();

        DriverFeedback feedback = new DriverFeedback(hardwareMap, driverGamepad, operatorGamepad, telemetry);

        driveTrain = new TeleFourWheelMecanumDriveTrain(hardwareMap, driverGamepad, telemetry, feedback);

        switchToMode(PowerMode.REGULAR);



        new Trigger(() -> gamepad1.right_trigger > 0.5)
                .whenActive(new InstantCommand(this::switchToNitroMode, driveTrain))
                .whenInactive(new InstantCommand(this::switchToRegularMode, driveTrain));

        new Trigger(() -> gamepad1.left_trigger > 0.5)
                .whenActive(new InstantCommand(this::switchToSlowMode, driveTrain))
                .whenInactive(new InstantCommand(this::switchToRegularMode, driveTrain));

        // Robot direction
        driverGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenHeld(new InstantCommand(driveTrain::ToggleDriveDirection, driveTrain));

        // Register all subsystems
        register(driveTrain, feedback);

        // update telemetry every loop
        schedule(SounderBotBaseRunCommand.createTelemetryEnabledOnlyInstance(telemetry));
    }

    @Override
    public void run() {
        super.run();
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