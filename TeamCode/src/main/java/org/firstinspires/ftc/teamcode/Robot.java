package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.arcrobotics.ftclib.command.RunCommand;
// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.opmodes.*;
import org.firstinspires.ftc.teamcode.subsystems.*;

public class Robot {

    private final OpMode opMode;

    private final GamepadEx driverGamepad;

    private final DriveSubsystem driveSubsystem;

    public Robot(OpMode opMode) {
        this.opMode = opMode;

        driverGamepad = new GamepadEx(opMode.gamepad1);
    
        driveSubsystem = new DriveSubsystem(opMode.hardwareMap);
    }

    public void configureTeleOpBindings() {
        
        /* Controls:
        * Driver:
        *   Forward -> left y axis
        *   Strafe -> left x axis
        *   Turn -> right x axis
        *
        *   Reduce Speed -> right trigger
        *   Reset Gyro -> back button
        *   Enable/Disable Field Centric -> start button
        */ 

        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().cancelAll();

        RunCommand defaultDriveCommand = new RunCommand(() -> driveSubsystem.drive(driverGamepad.getLeftY(), driverGamepad.getLeftX(), driverGamepad.getRightX()));
        defaultDriveCommand.addRequirements(driveSubsystem);

        driveSubsystem.setDefaultCommand(defaultDriveCommand);

        Trigger speedVariationTrigger = new Trigger(() -> isPressed(driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));
        speedVariationTrigger.whenActive(() -> driveSubsystem.changeSpeedMultiplier());
        //speedVariationTrigger.whenInactive(() -> driveSubsystem.setSpeedMultiplier(1));

        Trigger resetGyro = new Trigger(() -> driverGamepad.getButton(GamepadKeys.Button.BACK));
        resetGyro.whenActive(() -> driveSubsystem.resetGyro());

        Trigger setFieldCentric = new Trigger(() -> driverGamepad.getButton(GamepadKeys.Button.START));
        setFieldCentric.whenActive(() -> driveSubsystem.setFieldCentricOnOff());
    }

    public void configureAutoSetting(){
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().cancelAll();


    }

    public void run() {
        CommandScheduler.getInstance().run();
        opMode.telemetry.addData("Speed Multiplier",driveSubsystem.speedMultiplier);
        opMode.telemetry.addData("Y axis:", driverGamepad.getLeftY());
        opMode.telemetry.addData("Is fieldcentric?",driveSubsystem.fieldCentric);
        opMode.telemetry.update();
    }

    public  boolean isPressed(double controllerInput) {
        return Math.abs(controllerInput) >= Constants.DriveConstants.DEADZONE;
    }
}
