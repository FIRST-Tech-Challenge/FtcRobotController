package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.commandftc.opModes.CommandBasedTeleOp;
import org.firstinspires.ftc.teamcode.commands.DuckRoller.IndexDuckCommand;
import org.firstinspires.ftc.teamcode.commands.arm.RotateArmCommand;
import org.firstinspires.ftc.teamcode.commands.arm.RotateArmToPositionCommand;
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
    RotateArmToPositionCommand rotateArmToPositionCommandL;
    RotateArmToPositionCommand rotateArmToPositionCommandR;
    RotateArmCommand rotateArmCommand;
    // intake commands
    IntakeCommand intakeCommand;
    //Ducky command
    SequentialCommandGroup indexDuckCommand;

    private double getDriveSpeed() {
        if (gamepad1.left_trigger > 0)          return 0.5;
        else if (gamepad1.right_trigger > 0)    return 1;
        else                                    return 0.75;
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

        rotateArmToPositionCommandL = new RotateArmToPositionCommand(armSubsystem,1000,0.5);
        rotateArmToPositionCommandR = new RotateArmToPositionCommand(armSubsystem,-1000,0.5);

        rotateArmCommand = new RotateArmCommand(armSubsystem,
                () -> (gamepad2.left_trigger > gamepad2.right_trigger ?
                        gamepad2.left_trigger : -gamepad2.right_trigger)* 0.5);

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

        // Arm command
        armSubsystem.setDefaultCommand(rotateArmCommand);
        gp2.dpad_right().whileHeld(rotateArmToPositionCommandR);
        gp2.dpad_left().whileHeld(rotateArmToPositionCommandL);
        // Intake commands
        intakeSubsystem.setDefaultCommand(intakeCommand);
        // Duck commands
        gp2.y().whileHeld(indexDuckCommand);

        // Telemetry
        // No need for anything but update in loop because use of suppliers
        gp2.b().whenActive(() -> armSubsystem.setVerticalPosition(0.7), armSubsystem);
        gp2.a().whenActive(() -> armSubsystem.setVerticalPosition(0), armSubsystem);
        gp2.x().whenActive(() -> armSubsystem.setVerticalPosition(0.6), armSubsystem);



        telemetry.addData("Runtime", this::getRuntime);
        telemetry.addData("arm position", armSubsystem::getCurrentPosition);
        telemetry.addData("arm angle", armSubsystem::getAngle);
//        telemetry.addData("lift position", liftSubsystem::getCurrentPosition);
        telemetry.addData("duck position", ducksSubsystem::getCurrentPosition);
//        telemetry.addData("lift target", liftSubsystem::getTargetPosition);
//        telemetry.addData("lift current", liftSubsystem::getCurrentPosition);
        telemetry.update();
    }
}