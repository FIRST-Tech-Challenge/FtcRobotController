package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeArmSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;

@Autonomous(name = "AutonomousTesting", group = "Final Autonomous")
public class AutonomousTesting extends CommandOpMode {

    protected IntakeArmSubsystem intakeArmSubsystem;
    protected IntakeSubsystem intakeSubsystem;

    @Override
    public void initialize(){
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        intakeArmSubsystem = new IntakeArmSubsystem(hardwareMap);
    }

    @Override
    public void run() {
        initialize();
        waitForStart();

//        schedule(new SequentialCommandGroup(
//                new InstantCommand(intakeSubsystem::run, intakeSubsystem),
//                new InstantCommand(()-> intakeArmSubsystem.auto_pixel(5), intakeArmSubsystem),
//                new WaitCommand(300),
//                new InstantCommand(()-> intakeArmSubsystem.auto_pixel(4), intakeArmSubsystem),
//                new WaitCommand(3000),
//                new InstantCommand(()-> intakeArmSubsystem.auto_pixel(3), intakeArmSubsystem),
//                new WaitCommand(300),
//                new InstantCommand(()-> intakeArmSubsystem.auto_pixel(2), intakeArmSubsystem),
//                new WaitCommand(3000),
//                new InstantCommand(()-> intakeArmSubsystem.auto_pixel(1), intakeArmSubsystem)
//        ));

        schedule(new SequentialCommandGroup(
                new InstantCommand(intakeSubsystem::run, intakeSubsystem),
                new WaitCommand(150),
                new SequentialCommandGroup(
                        new InstantCommand(()-> intakeArmSubsystem.auto_pixel(5), intakeArmSubsystem),
                        new WaitCommand(1000),
                        new InstantCommand(()-> intakeArmSubsystem.auto_pixel(4), intakeArmSubsystem),
                        new WaitCommand(1000)
                ),
                new ParallelCommandGroup(
                        new InstantCommand(intakeSubsystem::stop, intakeSubsystem),
                        new InstantCommand(intakeArmSubsystem::raiseArm, intakeArmSubsystem)
                ),

                new WaitCommand(2000)
        ));

        while (!isStopRequested() && opModeIsActive()) {
            run();
        }

        reset();
    }
}
