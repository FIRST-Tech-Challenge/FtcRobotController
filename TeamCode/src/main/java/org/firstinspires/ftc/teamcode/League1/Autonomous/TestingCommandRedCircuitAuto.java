package org.firstinspires.ftc.teamcode.League1.Autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Common.Point;
import org.firstinspires.ftc.teamcode.League1.Common.Robot;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem;
import org.firstinspires.ftc.teamcode.League1.TeleOp.Command.LiftCommand;

public class TestingCommandRedCircuitAuto extends CommandOpMode {

    Robot robot;
    MecDrive drive;
    ScoringSystem score;
    Constants constants;


    @Override
    public void initialize() {

        constants = new Constants();
        robot = new Robot(hardwareMap);
        drive = new MecDrive(hardwareMap, robot, false, telemetry);
        score = new ScoringSystem(hardwareMap, robot, constants, false);
        robot.start();



        schedule(


                new SequentialCommandGroup(
                        new InstantCommand(() -> drive.newMoveToPosition(new Point(0, -1000), 0.5)),
                        new ParallelCommandGroup(
                                new InstantCommand(() -> drive.rotate(Math.PI/4, 0.2)),
                                new LiftCommand(score, constants, hardwareMap, robot, ScoringSystem.ExtensionHeight.HIGH, 0.5)
                        ),

                        new WaitCommand(1000), //score


                        new ParallelCommandGroup(
                                new InstantCommand(() -> drive.rotate(Math.PI/2, 0.2)),
                                new LiftCommand(score, constants, hardwareMap, robot, ScoringSystem.ExtensionHeight.ZERO, 0.5)
                        ),

                        new InstantCommand(() -> drive.newMoveToPosition(new Point(-1000, 0),0.5)),
                        new InstantCommand(() -> drive.newMoveToPosition(new Point(0, -1000), 0.5))
                )
        );


    }

    @Override
    public void run() {
        super.run();

        while(opModeIsActive()){

        }
    }
}
