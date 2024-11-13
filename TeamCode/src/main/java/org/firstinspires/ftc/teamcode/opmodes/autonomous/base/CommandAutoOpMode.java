package org.firstinspires.ftc.teamcode.opmodes.autonomous.base;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.command.CommandFactory;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliveryPivot;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliverySlider;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;
import org.firstinspires.ftc.teamcode.subsystems.intake.RollingIntake;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimeLight;

import java.util.List;

public abstract class CommandAutoOpMode extends CommandOpMode {
    protected CommandFactory commandFactory;

    @Override
    public void initialize() {
        GamepadEx driverGamePad = new GamepadEx(gamepad1);
        GamepadEx operatorGamePad = new GamepadEx(gamepad2);
        DriverFeedback feedback = new DriverFeedback(hardwareMap, driverGamePad, operatorGamePad, telemetry);
        LimeLight limeLight = new LimeLight(hardwareMap, telemetry);
        AutoMecanumDriveTrain driveTrain = new AutoMecanumDriveTrain(hardwareMap, driverGamePad, telemetry, feedback, limeLight);
        RollingIntake rollingIntake = new RollingIntake(hardwareMap, operatorGamePad, telemetry, feedback);
        DeliveryPivot pivot = new DeliveryPivot(hardwareMap, operatorGamePad, telemetry, feedback);
        DeliverySlider slider = new DeliverySlider(hardwareMap, operatorGamePad, telemetry, feedback);
        commandFactory = new CommandFactory(telemetry, driveTrain, rollingIntake, limeLight, pivot, slider);
        schedule(createCommand());
    }

    protected abstract Command createCommand();
}
