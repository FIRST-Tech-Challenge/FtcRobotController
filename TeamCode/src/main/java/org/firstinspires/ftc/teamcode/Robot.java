package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

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

       driveSubsystem = new DriveSubsystem(opMode);
       intakeSubsystem = new IntakeSubsystem(opMode);
       elbowSubsystem = new ElbowSubsystem(opMode);
       linearSlideSubsystem = new LinearSlideSubsystem(opMode);
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
       *    Go to start from intaking -> left d-pad
       *    Go to outtake position -> up d-pad
       *    Go to start from outtaking -> down d-pad
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

       LinearSlideSubsystem.RunPIDCommand defaultLinearSlideCommand = new LinearSlideSubsystem.RunPIDCommand(linearSlideSubsystem);
       defaultLinearSlideCommand.addRequirements(linearSlideSubsystem);
       linearSlideSubsystem.setDefaultCommand(defaultLinearSlideCommand);

       ElbowSubsystem.RunPIDCommand defaultElbowCommand = new ElbowSubsystem.RunPIDCommand(elbowSubsystem);
       defaultElbowCommand.addRequirements(elbowSubsystem);
       elbowSubsystem.setDefaultCommand(defaultElbowCommand);

       Trigger setElbowPosition = new Trigger(() -> operatorGamepad.getLeftY() != 0);
       setElbowPosition.whileActiveContinuous(() -> elbowSubsystem.setTargetPosition(elbowSubsystem.convertStickToTarget(operatorGamepad.getLeftY(), Constants.IntakeConstants.MANUAL_CONTROL_RATE)));

       Trigger setLinearSlidePosition = new Trigger(() -> operatorGamepad.getRightY() != 0);
       setLinearSlidePosition.whileActiveContinuous(() -> linearSlideSubsystem.setTargetPosition(linearSlideSubsystem.convertStickToTarget(operatorGamepad.getRightY(), Constants.IntakeConstants.MANUAL_CONTROL_RATE)));

       Trigger moveToIntakePosition = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER));
       moveToIntakePosition.whenActive(() -> intakeSubsystem.rotateToPosition(IntakeSubsystem.WristPositions.INTAKE_POSITION));

       Trigger moveToOuttakePosition = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER));
       moveToOuttakePosition.whenActive(() -> intakeSubsystem.rotateToPosition(IntakeSubsystem.WristPositions.OUTTAKE_POSITION));

       Trigger intake = new Trigger(() -> operatorGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > Constants.DriveConstants.DEADZONE);
       intake.whenActive(() -> intakeSubsystem.intake());
       intake.whenInactive(() -> intakeSubsystem.stopIntakeServo());

       Trigger outtake = new Trigger(() -> operatorGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > Constants.DriveConstants.DEADZONE);
       intake.whenActive(() -> intakeSubsystem.outtake());
       intake.whenInactive(() -> intakeSubsystem.stopIntakeServo());

       Trigger goHome = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.DPAD_LEFT));
       goHome.whenActive(new SequentialCommandGroup(
               new RunCommand(() -> elbowSubsystem.setTargetPosition(Constants.IntakeConstants.ELBOW_STARTING_POS)),
               new RunCommand(() -> linearSlideSubsystem.setTargetPosition(Constants.IntakeConstants.LINEAR_STARTING_POS))
       ));

       Trigger goToOuttakePosition = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.DPAD_UP));
       goToOuttakePosition.whenActive(new SequentialCommandGroup(
               new RunCommand(() -> elbowSubsystem.setTargetPosition(Constants.IntakeConstants.MAXIMUM_ELBOW_POS)),
               new RunCommand(() -> linearSlideSubsystem.setTargetPosition(Constants.IntakeConstants.MAXIMUM_SLIDE_POS))
       ));

       Trigger goHomeFromOuttake = new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.DPAD_DOWN));
       goHomeFromOuttake.whenActive(new SequentialCommandGroup(
               new RunCommand(() -> linearSlideSubsystem.setTargetPosition(Constants.IntakeConstants.LINEAR_STARTING_POS)),
               new RunCommand(() -> elbowSubsystem.setTargetPosition(Constants.IntakeConstants.ELBOW_STARTING_POS))
       ));
   }

   public void run() {
       CommandScheduler.getInstance().run();
       opMode.telemetry.addData("Linear Slide Position:", linearSlideSubsystem.getCurrentPosition());
       opMode.telemetry.addData("Elbow Current Position:", elbowSubsystem.getCurrentPosition());
       opMode.telemetry.addData("Wrist Current Position:", intakeSubsystem.getCurrentPosition());
       opMode.telemetry.update();
   }
}