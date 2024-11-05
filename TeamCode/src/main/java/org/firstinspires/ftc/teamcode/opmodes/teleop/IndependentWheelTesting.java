package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.command.SounderBotBaseRunCommand;
import org.firstinspires.ftc.teamcode.opmodes.OpModeTemplate;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.IndependentCtrlWheelMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;

//@TeleOp (name = "wheel_test")
@SuppressWarnings("unused")
public class IndependentWheelTesting extends OpModeTemplate {

    private IndependentCtrlWheelMecanumDriveTrain driveTrain;

    @Override
    public void initialize() {
        super.initialize();

        DriverFeedback feedback = new DriverFeedback(hardwareMap, driverGamepad, operatorGamepad, telemetry);

        driveTrain = new IndependentCtrlWheelMecanumDriveTrain(hardwareMap, driverGamepad, telemetry, feedback);

        driveTrain.setPowerRatio(0.5);

        // Robot direction
        driverGamepad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(driveTrain::onYPressed));

        driverGamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(driveTrain::onXPressed));

        driverGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(driveTrain::onAPressed));

        driverGamepad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(driveTrain::onBPressed));

        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(driveTrain::allStop));

        // Register all subsystems
        register(driveTrain, feedback);

        // update telemetry every loop
        schedule(SounderBotBaseRunCommand.createTelemetryEnabledOnlyInstance(telemetry));
    }

    @Override
    public void run() {
        super.run();
    }
}