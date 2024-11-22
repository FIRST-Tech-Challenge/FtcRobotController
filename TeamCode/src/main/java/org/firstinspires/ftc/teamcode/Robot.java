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
        intakeSubsystem = new IntakeSubsystem(opMode.hardwareMap,opMode);
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

        /* Operator controls
        Reset linear slide encoders -> Start button
        Reset arm joint encoders -> Back button

        Put down arm + enable right y-axis control + intake wrist to be parallel with the floor -> Y
        Pull up arm + linear slide & intake wrist to high basket position -> A
        Pull up arm + linear slide & intake wrist to get specimen position -> X
        Pull up arm + linear slide & intake wrist to hang specimen position -> B

        Rest arm + retract linear slide & intake wrist -> left joy stick button

        active intake in/stop -> right bumper(toggle)
        active intake out -> right trigger(continuous)
         */
        Trigger resetLinearSlideEncoder = new Trigger(()-> operatorGamepad.getButton(GamepadKeys.Button.BACK));
        resetLinearSlideEncoder.whenActive(()->intakeSubsystem.resetLinearSlideEncoders());

        Trigger resetArmJointEncoder = new Trigger(()-> operatorGamepad.getButton(GamepadKeys.Button.START));
        resetArmJointEncoder.whenActive(()-> intakeSubsystem.resetArmJointEncoders());

        RunCommand defaultFloorScanningCursorCommand = new RunCommand(() -> intakeSubsystem.manualFloorScanningMode(operatorGamepad.getRightY()));
        defaultFloorScanningCursorCommand.addRequirements(intakeSubsystem);

        // Gamepad 1 (Operator Gamepad)

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

        Trigger activeIntakeIn = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER));
        activeIntakeIn.whileActiveContinuous(() -> intakeSubsystem.ActiveIntakeServoIn());
        activeIntakeIn.whenInactive(()-> intakeSubsystem.stopActiveIntakeServo());

        Trigger activeIntakeOut = new Trigger(() -> isPressed(operatorGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));
        activeIntakeOut.whenActive(() -> intakeSubsystem.ActiveIntakeServoOut());

        intakeSubsystem.setDefaultCommand(defaultFloorScanningCursorCommand);

    }

    public void configureIntakeTestingTeleOpBindings() {

        /* Driver controls:
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
        speedVariationTrigger.whenActive(() -> driveSubsystem.changeSpeedMultiplier());//feedback from driver: changed the speed multiplier to left trigger and changed by toggle
        //speedVariationTrigger.whenInactive(() -> driveSubsystem.setSpeedMultiplier(1));

        Trigger resetGyro = new Trigger(() -> driverGamepad.getButton(GamepadKeys.Button.BACK));
        resetGyro.whenActive(() -> driveSubsystem.resetGyro());

        Trigger setFieldCentric = new Trigger(() -> driverGamepad.getButton(GamepadKeys.Button.START));
        setFieldCentric.whenActive(() -> driveSubsystem.setFieldCentricOnOff());

        /* Operator controls
        Reset linear slide encoders -> Back button
        Reset arm joint encoders -> Start button

        Linear slide manual control -> right y-axis
        Arm Joint manual control -> left y-axis

        put down intake wrist -> Y button
        pull up intake wrist -> A button

        active intake in/stop -> right bumper(toggle)
        active intake out -> right trigger(continuous)
         */
        Trigger resetLinearSlideEncoder = new Trigger(()-> operatorGamepad.getButton(GamepadKeys.Button.BACK));
        resetLinearSlideEncoder.whenActive(()->intakeSubsystem.resetLinearSlideEncoders());

        Trigger resetArmJointEncoder = new Trigger(()-> operatorGamepad.getButton(GamepadKeys.Button.START));
        resetArmJointEncoder.whenActive(()-> intakeSubsystem.resetArmJointEncoders());

        RunCommand defaultLinearSlideCommand = new RunCommand(() -> intakeSubsystem.runLinearSlideManually(operatorGamepad.getRightY()));
        defaultLinearSlideCommand.addRequirements(intakeSubsystem);

        intakeSubsystem.setDefaultCommand(defaultLinearSlideCommand);

        RunCommand defaultArmJointCommand = new RunCommand(() -> intakeSubsystem.runArmJointsManually(operatorGamepad.getLeftY()));
        defaultArmJointCommand.addRequirements(intakeSubsystem);

        intakeSubsystem.setDefaultCommand(defaultArmJointCommand);

        Trigger putDownIntakeWrist = new Trigger(()-> operatorGamepad.getButton(GamepadKeys.Button.Y));
        putDownIntakeWrist.whenActive(()->intakeSubsystem.servoDownPosition());

        Trigger pullUpIntakeWrist = new Trigger(()-> operatorGamepad.getButton(GamepadKeys.Button.A));
        pullUpIntakeWrist.whenActive(()->intakeSubsystem.servoUpPosition());

        Trigger activeIntakeIn = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.B));
        activeIntakeIn.whileActiveContinuous(() -> intakeSubsystem.ActiveIntakeServoIn());
        activeIntakeIn.whenInactive(()-> intakeSubsystem.stopActiveIntakeServo());

        Trigger activeIntakeOut = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.X));
        activeIntakeOut.whileActiveContinuous(() -> intakeSubsystem.ActiveIntakeServoOut());
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
