package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.arcrobotics.ftclib.command.RunCommand;

import org.firstinspires.ftc.teamcode.opmodes;
import org.firstinspires.ftc.teamcode.subsystems;

public class Robot {

    private final OpMode opMode;

    private final GamepadEx driverGamepad;

    private final DriveSubsystem driveSubsystem;

    public Robot(OpMode opMode) {
        this.opMode = opMode;

        driverGamepad = new GamepadEx(opMode.gamepad1);
    
        driveSubsystem = new DriveSubsystem(opMode.hardwareMap, opMode.telemetry);
    }

    public void configureTeleOpBindings() {
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().cancelAll();

        driveSubsystem.setDefaultCommand(new RunCommand(driveSubsystem.drive(driverGamepad.getLeftY(), driverGamepad.getLeftX(), driverGamepad.getRightX())));

        Trigger speedVariationTrigger = new Trigger(() -> isPressed(driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));
        speedVariationTrigger.whenActive(driveSubsystem.setSpeedMultiplier(driverGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) * 0.8 + 0.2));
    }

    public void run() {
        CommandScheduler.getInstance().run();
        opMode.telemetry.update();
    }

    public  boolean isPressed(double controllerInput) {
        return Math.abs(controllerInput) >= Constants.DriveConstants.DEADZONE;
    }

}
