package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import org.firstinspires.ftc.teamcode.subsystems.*;

public class Robot {
    private final OpMode opMode;

    private final GamepadEx driverGamepad;
    private final GamepadEx operatorGamepad;

    private final DriveSubsystem driveSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public Robot(OpMode opMode) {
        this.opMode = opMode;

        driverGamepad = new GamepadEx(opMode.gamepad1);
        operatorGamepad = new GamepadEx(opMode.gamepad2);
    
        driveSubsystem = new DriveSubsystem(opMode.hardwareMap);
        intakeSubsystem = new IntakeSubsystem(opMode.hardwareMap, opMode);
    }

    public void configureTeleOpBindings() {
        /* Driver controls:
        *   Forward -> left y axis
        *   Strafe -> left x axis
        *   Turn -> right x axis
        *
        *   Reduce Speed -> right trigger
        *   Reset Gyro -> back button
        *   Enable/Disable Field Centric -> start button
        *
        * Operator:
         *   Intake subsystem:
         *      Extend horizontal slide to max + intake wrist down -> Y
         *      Retract horizontal slide to min + intake wrist up -> A
         *      Active intake in -> B(cont.)
         *      Active intake out -> A(cont.)
         *
         *      reset horizontal slide encoder -> start
        */ 

        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().cancelAll();

        //Driver Controls

        RunCommand defaultDriveCommand = new RunCommand(() -> driveSubsystem.drive(driverGamepad.getLeftY(), driverGamepad.getLeftX(), driverGamepad.getRightX()));
        defaultDriveCommand.addRequirements(driveSubsystem);
        driveSubsystem.setDefaultCommand(defaultDriveCommand);

        RunCommand defaultFloorScanningCursorCommand = new RunCommand(() -> intakeSubsystem.manualFloorScanningMode(operatorGamepad.getRightY()));
        defaultFloorScanningCursorCommand.addRequirements(intakeSubsystem);
        intakeSubsystem.setDefaultCommand(defaultFloorScanningCursorCommand);

        Trigger speedVariationTrigger = new Trigger(() -> driverGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER));
        speedVariationTrigger.whenActive(() -> driveSubsystem.setSpeedMultiplier(0.2));
        speedVariationTrigger.whenInactive(() -> driveSubsystem.setSpeedMultiplier(1));

        Trigger resetGyro = new Trigger(() -> driverGamepad.getButton(GamepadKeys.Button.BACK));
        resetGyro.whenActive(() -> driveSubsystem.resetGyro());

        Trigger setFieldCentric = new Trigger(() -> driverGamepad.getButton(GamepadKeys.Button.B));
        setFieldCentric.whenActive(() -> driveSubsystem.setFieldCentricOnOff());

        //Operator Controls

        Trigger activeIntakeIn = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.B));
        activeIntakeIn.whileActive(()-> intakeSubsystem.activeIntakeServoIn());
        activeIntakeIn.whenInactive(()-> intakeSubsystem.stopActiveIntakeServo());

        Trigger activeIntakeOut = new Trigger(()-> operatorGamepad.getButton(GamepadKeys.Button.X));
        activeIntakeOut.whileActive(()->intakeSubsystem.activeIntakeServoOut());
        activeIntakeOut.whenInactive(()-> intakeSubsystem.stopActiveIntakeServo());

        Trigger buttonA = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.A));
        buttonA.whenActive(() -> {
            intakeSubsystem.changeState(Constants.IntakeConstants.IntakeSubsystemState.HIGH_BASKET_POSITION);
        });

        Trigger buttonB = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.B));
        buttonB.whenActive(() -> {
            intakeSubsystem.changeState(Constants.IntakeConstants.IntakeSubsystemState.HANG_SPECIMEN_POSITION);
        });

        Trigger buttonX = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.X));
        buttonX.whenActive(() -> {
            intakeSubsystem.changeState(Constants.IntakeConstants.IntakeSubsystemState.GET_SPECIMEN_POSITION);
        });

        Trigger buttonY = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.Y));
        buttonY.whenActive(() -> {
            intakeSubsystem.changeState(Constants.IntakeConstants.IntakeSubsystemState.FLOOR_SCANNING_MODE);
        });

        Trigger atbay = new Trigger(()-> operatorGamepad.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON));
        atbay.whenActive(()-> intakeSubsystem.changeState(Constants.IntakeConstants.IntakeSubsystemState.AT_BAY));
    }

    public void configureAutoSetting() {
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().cancelAll();
    }

    public void run() {
        CommandScheduler.getInstance().run();

        opMode.telemetry.addData("Speed Multiplier",driveSubsystem.getSpeedMultiplier());
        opMode.telemetry.addData("Y axis:", operatorGamepad.getLeftY());
        opMode.telemetry.addData("Is fieldcentric?",driveSubsystem.getIsFieldCentric());
        opMode.telemetry.addData("value: ", intakeSubsystem.getDistanceTraveled());
        opMode.telemetry.update();
    }
}