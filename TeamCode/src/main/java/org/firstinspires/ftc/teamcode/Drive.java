package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.commandftc.opModes.CommandBasedTeleOp;
import org.firstinspires.ftc.teamcode.commands.DuckRoller.IndexDuckCommand;
import org.firstinspires.ftc.teamcode.commands.arm.RotateArmCommand;
import org.firstinspires.ftc.teamcode.commands.drive.ArcadeDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drive.DriveLeftCommand;
import org.firstinspires.ftc.teamcode.commands.drive.DriveRightCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TankDriveCommand;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DucksSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

@TeleOp(name = "Drive")
public class Drive extends CommandBasedTeleOp
{
    DriveTrainSubsystem driveTrain;
    LiftSubsystem liftSubsystem;
    ArmSubsystem armSubsystem;
    IntakeSubsystem intakeSubsystem;
    DucksSubsystem ducksSubsystem;
    // Drive Commands
    TankDriveCommand tankDriveCommand;
    ArcadeDriveCommand arcadeDriveCommand;
    DriveLeftCommand driveLeftCommand;
    DriveRightCommand driveRightCommand;
    // Lift Commands

    // Arm commands
    RotateArmCommand rotateArmCommand;
    Command rotateArmLeftCommand;
    Command rotateArmRightCommand;
    Command centerArmCommand;
    Command rotateArmContinuouslyCommand;
    // intake commands
    IntakeCommand intakeCommand;
    //Ducky command
    SequentialCommandGroup indexDuckCommand;

    private double getDriveSpeed() {
        if (gamepad1.left_trigger > 0)          return 0.5;
        else if (gamepad1.right_trigger > 0)    return 1;
        else                                    return 0.75;
    }

    private double getArmRotationSide(){
        if(gamepad2.left_trigger - gamepad2.right_trigger > 0)      return 1;
        else if(gamepad2.left_trigger - gamepad2.right_trigger < 0) return -1;
        else                                                        return 0;
    }

    @Override
    public void assign() {
        driveTrain = new DriveTrainSubsystem();
        liftSubsystem = new LiftSubsystem();
        armSubsystem = new ArmSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        ducksSubsystem = new DucksSubsystem();

        tankDriveCommand = new TankDriveCommand(driveTrain, () -> -gamepad1.left_stick_y * getDriveSpeed(), () -> -gamepad1.right_stick_y * getDriveSpeed());
        arcadeDriveCommand = new ArcadeDriveCommand(driveTrain, () -> gamepad1.left_stick_x, () -> -gamepad1.left_stick_y, () -> gamepad1.right_stick_x);
        driveLeftCommand = new DriveLeftCommand(driveTrain, this::getDriveSpeed);
        driveRightCommand = new DriveRightCommand(driveTrain, this::getDriveSpeed);

        rotateArmCommand = new RotateArmCommand(armSubsystem, 1, 2, new double[]{-85, -45, 0, 45, 85});
        rotateArmLeftCommand = new InstantCommand(rotateArmCommand::decState);
        rotateArmRightCommand = new InstantCommand(rotateArmCommand::incState);
        centerArmCommand = new InstantCommand(() -> rotateArmCommand.setState(2));
        rotateArmContinuouslyCommand = new InstantCommand(() ->armSubsystem.setAngle(armSubsystem.getAngle() + getArmRotationSide()));
        intakeCommand = new IntakeCommand(intakeSubsystem, () -> gamepad2.right_stick_y);
        // TODO: pulse power spin till duck falls and wait for new duck
        indexDuckCommand = new IndexDuckCommand(ducksSubsystem,1000,1).andThen(new WaitCommand(5));

        // DriveTrain commands
        driveTrain.setDefaultCommand(tankDriveCommand);
        gp1.x().whileHeld(arcadeDriveCommand);
        gp1.dpad_left().whileHeld(driveLeftCommand);
        gp1.left_bumper().whileHeld(driveLeftCommand);
        gp1.dpad_right().whileHeld(driveRightCommand);
        gp1.right_bumper().whileHeld(driveRightCommand);
        // Lift commands
        // TODO: implement lift control using position
        CommandBase liftCommand = new CommandBase() {
            @Override
            public void execute() {
                liftSubsystem.setPower(-gamepad1.left_stick_y);
            }
        };
        liftCommand.addRequirements(liftSubsystem);
        liftSubsystem.setDefaultCommand(liftCommand);
        // Arm command
        armSubsystem.setDefaultCommand(rotateArmCommand);

        gp2.dpad_left().whenPressed(rotateArmLeftCommand);
        gp2.dpad_right().whenPressed(rotateArmRightCommand);
        gp2.dpad_down().whenPressed(centerArmCommand);

        gp2.left_bumper().whileHeld(() -> armSubsystem.setVerticalPosition(Math.min(armSubsystem.getVerticalPosition()+0.02, 0.7)), armSubsystem);
        gp2.right_bumper().whileHeld(() -> armSubsystem.setVerticalPosition(Math.max(armSubsystem.getVerticalPosition()-0.02, 0)), armSubsystem);

        gp2.b().whenActive(() -> armSubsystem.setVerticalPosition(0.7), armSubsystem);
        gp2.a().whenActive(() -> armSubsystem.setVerticalPosition(0), armSubsystem);
        gp2.x().whenActive(() -> armSubsystem.setVerticalPosition(0.6), armSubsystem);
        // Intake commands
        intakeSubsystem.setDefaultCommand(intakeCommand);
        // Duck commands
        gp2.y().whileHeld(indexDuckCommand);

        // Telemetry
        // No need for anything but update in loop because use of suppliers
        telemetry.addData("Runtime", this::getRuntime);
        telemetry.addData("dt(s)", this::dt);
        telemetry.addData("arm position", armSubsystem::getCurrentPosition);
        telemetry.addData("arm angle", armSubsystem::getAngle);
        telemetry.addData("arm state", rotateArmCommand::getState);
        telemetry.addData("arm state[calc]", rotateArmCommand::calculateState);
        telemetry.addData("duck position", ducksSubsystem::getCurrentPosition);
        telemetry.addData("Arm Rotation Side", this::getArmRotationSide);
        telemetry.update();
    }
}