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
   private final ElbowSubsystem elbowSubsystem;
   private final LinearSlideSubsystem linearSlideSubsystem;

   public Robot(OpMode opMode) {
       this.opMode = opMode;

       driverGamepad = new GamepadEx(opMode.gamepad1);
       operatorGamepad = new GamepadEx(opMode.gamepad2);

       driveSubsystem = new DriveSubsystem(opMode.hardwareMap);
       intakeSubsystem = new IntakeSubsystem(opMode.hardwareMap);
       elbowSubsystem = new ElbowSubsystem(opMode.hardwareMap);
       linearSlideSubsystem = new LinearSlideSubsystem(opMode.hardwareMap);
   }

   public void configureTeleOpBindings() {
       /* Driver controls:
       *   Forward -> left y axis
       *   Strafe -> left x axis
       *   Turn -> right x axis
       *
       *   Reduce Speed -> right bumper
       *   Reset Gyro -> back button
       *   Enable/Disable Field Centric -> b
       *
       * Operator:
       *    Move Linear Slide -> right y axis
       *    Move Elbow -> left y axis
       * 
       *    Go to Intake Position -> right bumper
       *    Go to Outtake Position -> left bumper
       *    Intake -> right trigger
       *    Outtake -> left trigger
       */

       CommandScheduler.getInstance().reset();
       CommandScheduler.getInstance().cancelAll();

       //Driver Controls

       RunCommand defaultDriveCommand = new RunCommand(() -> driveSubsystem.drive(driverGamepad.getLeftY(), driverGamepad.getLeftX(), driverGamepad.getRightX()));
       defaultDriveCommand.addRequirements(driveSubsystem);
       driveSubsystem.setDefaultCommand(defaultDriveCommand);

       Trigger speedVariationTrigger = new Trigger(() -> driverGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER));
       speedVariationTrigger.whenActive(() -> driveSubsystem.setSpeedMultiplier(0.2));
       speedVariationTrigger.whenInactive(() -> driveSubsystem.setSpeedMultiplier(1));

       Trigger resetGyro = new Trigger(() -> driverGamepad.getButton(GamepadKeys.Button.BACK));
       resetGyro.whenActive(() -> driveSubsystem.resetGyro());

       Trigger setFieldCentric = new Trigger(() -> driverGamepad.getButton(GamepadKeys.Button.B));
       setFieldCentric.whenActive(() -> driveSubsystem.setFieldCentricOnOff());

       //Operator Controls

       LinearSlideSubsystem.RunPIDCommand defaultLinearSlideCommand = new LinearSlideSubsystem.RunPIDCommand(linearSlideSubsystem.convertStickToTarget(operatorGamepad.getRightY(), 2), linearSlideSubsystem);
       defaultLinearSlideCommand.addRequirements(linearSlideSubsystem);
       linearSlideSubsystem.setDefaultCommand(defaultLinearSlideCommand);

       ElbowSubsystem.RunPIDCommand defaultElbowCommand = new ElbowSubsystem.RunPIDCommand(elbowSubsystem.convertStickToTarget(operatorGamepad.getLeftY, 2), elbowSubsystem);
       defaultElbowCommand.addRequirements(elbowSubsystem);
       elbowSubsystem.setDefaultCommand(defaultElbowCommand);

       Trigger moveToIntakePosition = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER));
       moveToIntakePosition.whenActive(() -> intakeSubsystem.rotateToPosition(IntakeSubsystem.WristPositions.INTAKE_POSITION));

       Trigger moveToOuttakePosition = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER));
       moveToOuttakePosition.whenActive(() -> intakeSubsystem.rotateToPosition(IntakeSubsystem.WristPositions.OUTTAKE_POSITION));

       Trigger intake = new Trigger(() -> operatorGamepad.getRightTrigger() > Constants.DriveConstants.DEADZONE);
       intake.whenActive(() -> intakeSubsystem.intake());
       intake.whenInactive(() -> intakeSubsystem.stopIntakeServo());

       Trigger outtake = new Trigger(() -> operatorGamepad.getLeftTrigger() > Constants.DriveConstants.DEADZONE);
       intake.whenActive(() -> intakeSubsystem.outtake());
       intake.whenInactive(() -> intakeSubsystem.stopIntakeServo());
   }

   public void run() {
       CommandScheduler.getInstance().run();
   }
}