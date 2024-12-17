package org.firstinspires.ftc.teamcode.opmodes.AutonomousOpmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PincherSubsystem;
import org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer;

@Autonomous(name = "Simple Park (WIP)", group = "Autonomous")
public class MoveAutonomous extends CommandOpMode {

    DriveSubsystem driveSubsystem;
    PincherSubsystem pincherSubsystem;
    DefaultDrive driveCommand;

    @Override
    public void initialize() {
        try {
            driveSubsystem = new DriveSubsystem(hardwareMap);
            register(driveSubsystem);
            driveCommand = new DefaultDrive(driveSubsystem, gamepad1);
            //driveSubsystem.setDefaultCommand(driveCommand);
        } catch (Exception e) {
            e.printStackTrace();
            requestOpModeStop();
            return;
        }

        try {
            ServoEx pincher1 = RobotHardwareInitializer.ServoComponent.FINGER_1.getEx(hardwareMap, 0, PincherSubsystem.MAX_ANGLE);
            ServoEx pincher2 = RobotHardwareInitializer.ServoComponent.FINGER_2.getEx(hardwareMap, 0, PincherSubsystem.MAX_ANGLE);
            pincherSubsystem  = new PincherSubsystem(pincher1, pincher2);
            register(pincherSubsystem);
        } catch (Exception e) {
            e.printStackTrace();
            requestOpModeStop();
            return;
        }

        waitForStart();

        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
                new WaitCommand(2605), // pause before moving
                new RunCommand(() -> {
                   telemetry.addData("ITS MOVING NOW", "YAY");
                   telemetry.update();
                }),
                new DriveCommand(driveSubsystem, 0, .5f, 0),
                new WaitCommand(250), // move away from wall for a bit
                new DriveCommand(driveSubsystem, 0, 0, 0),
                new WaitCommand(500), // wait before moving forward
                new DriveCommand(driveSubsystem, 1, 0, 0),
                new WaitCommand(500),
                new DriveCommand(driveSubsystem, 0, 0, 0)
        ) {
            @Override
            public boolean isFinished() {
                return false;
            }
        };
        driveSubsystem.setDefaultCommand(commandGroup);
        pincherSubsystem.closeFinger();
    }
}
