package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.arcrobotics.ftclib.command.RunCommand;
// RR-specific imports

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

        intakeSubsystem = new IntakeSubsystem(opMode.hardwareMap,opMode);
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

        RunCommand defaultDriveCommand = new RunCommand(() -> driveSubsystem.drive(driverGamepad.getLeftY(), driverGamepad.getLeftX(), driverGamepad.getRightX()));
        defaultDriveCommand.addRequirements(driveSubsystem);

        driveSubsystem.setDefaultCommand(defaultDriveCommand);

        Trigger speedVariationTrigger = new Trigger(() -> isPressed(driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));
        speedVariationTrigger.whenActive(() -> driveSubsystem.changeSpeedMultiplier());//feedback from driver: changed the speed multiplier to left trigger and changed by toggle
        //speedVariationTrigger.whenInactive(() -> driveSubsystem.setSpeedMultiplier(1));

        Trigger resetGyro = new Trigger(() -> driverGamepad.getButton(GamepadKeys.Button.BACK));
        resetGyro.whenActive(() -> driveSubsystem.resetGyro());

        Trigger setFieldCentric = new Trigger(() -> driverGamepad.getButton(GamepadKeys.Button.B));
        setFieldCentric.whenActive(() -> driveSubsystem.setFieldCentricOnOff());

        Trigger activeIntakeIn = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.B));
        activeIntakeIn.whileActiveContinuous(()-> intakeSubsystem.ActiveIntakeServoIn());
        activeIntakeIn.whenInactive(()-> intakeSubsystem.stopActiveIntakeServo());

        Trigger activeIntakeOut = new Trigger(()-> operatorGamepad.getButton(GamepadKeys.Button.X));
        activeIntakeOut.whileActiveContinuous(()->intakeSubsystem.ActiveIntakeServoOut());
        activeIntakeOut.whenInactive(()-> intakeSubsystem.stopActiveIntakeServo());


    }

    public void configureSubsystemTestingTeleOpBindings() {

        /* Controls:
         * Driver:
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
         *      Put intake wrist down -> right trigger(continuously)
         *      Active intake in -> B(cont.)
         *      Active intake out -> A(cont.)
         *
         */

        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().cancelAll();

        RunCommand defaultDriveCommand = new RunCommand(() -> driveSubsystem.drive(driverGamepad.getLeftY(), driverGamepad.getLeftX(), driverGamepad.getRightX()));
        defaultDriveCommand.addRequirements(driveSubsystem);

        driveSubsystem.setDefaultCommand(defaultDriveCommand);

        Trigger speedVariationTrigger = new Trigger(() -> isPressed(driverGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));
        speedVariationTrigger.whenActive(() -> driveSubsystem.changeSpeedMultiplier());//feedback from driver: changed the speed multiplier to left trigger and changed by toggle
        //speedVariationTrigger.whenInactive(() -> driveSubsystem.setSpeedMultiplier(1));

        Trigger resetGyro = new Trigger(() -> driverGamepad.getButton(GamepadKeys.Button.BACK));
        resetGyro.whenActive(() -> driveSubsystem.resetGyro());

        Trigger setFieldCentric = new Trigger(() -> driverGamepad.getButton(GamepadKeys.Button.B));
        setFieldCentric.whenActive(() -> driveSubsystem.setFieldCentricOnOff());

        // Intake bindings,revised

        Trigger activeIntakeIn = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.B));
        activeIntakeIn.whileActiveContinuous(() -> intakeSubsystem.ActiveIntakeServoIn());
        activeIntakeIn.whenInactive(()-> intakeSubsystem.stopActiveIntakeServo());

        Trigger activeIntakeOut = new Trigger(() ->operatorGamepad.getButton(GamepadKeys.Button.X));
        activeIntakeOut.whileActiveContinuous(() -> intakeSubsystem.ActiveIntakeServoOut());
        activeIntakeOut.whenInactive(()-> intakeSubsystem.stopActiveIntakeServo());

        Trigger turnIntakeWrist = new Trigger(() -> isPressed(operatorGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));
        turnIntakeWrist.whileActiveContinuous(() -> intakeSubsystem.servoDownPosition());
        turnIntakeWrist.whenInactive(()-> intakeSubsystem.servoUpPosition());

    }

    public void configureAutoSetting(){
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

    public  boolean isPressed(double controllerInput) {
        return Math.abs(controllerInput) >= Constants.DriveConstants.DEADZONE;
    }
}
