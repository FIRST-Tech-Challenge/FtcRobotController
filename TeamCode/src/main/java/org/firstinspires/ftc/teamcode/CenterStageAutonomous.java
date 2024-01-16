package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CenterStageRobot.commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.OuttakeSusystem;
import org.firstinspires.ftc.teamcode.Extra.CommandAction;
import org.firstinspires.ftc.teamcode.Extra.CommandGroupBaseAction;

import java.util.function.DoubleSupplier;

@Autonomous(name = "CenterStageAutonomous", group = "Final Autonomous")
public class CenterStageAutonomous extends LinearOpMode {

    protected MecanumDrive drive;
    protected RoadRunnerSubsystem RR;
    protected OuttakeSusystem outtakeSusystem;
    protected ElevatorSubsystem elevatorSubsystem;
    protected ElevatorCommand elevatorCommand;

    protected Pose2d homePose_LOW_RED = new Pose2d((3 * RR.TileInverted) + (RR.RobotY/2),(RR.TileInverted/2),Math.toRadians(180));
    protected Pose2d homePose_HIGH_RED = new Pose2d((3 * RR.TileInverted) + (RR.RobotY/2),(RR.Tile * 1.5),Math.toRadians(180));

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, homePose_LOW_RED);
        RR = new RoadRunnerSubsystem(drive, RoadRunnerSubsystem.Alliance.RED, RoadRunnerSubsystem.Start.LOW,
                RoadRunnerSubsystem.Corridor.INNER, RoadRunnerSubsystem.Corridor.INNER,
                RoadRunnerSubsystem.Station.INNER, RoadRunnerSubsystem.Parking.OUTER);

        outtakeSusystem = new OuttakeSusystem(hardwareMap);
        elevatorSubsystem = new ElevatorSubsystem(hardwareMap, telemetry, () -> 0);

                waitForStart();

        Actions.runBlocking(new SequentialAction(
                RR.HIGH_HomeToPixel_CENTER.build(),
                new ParallelAction(
                        new CommandAction(
                                new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.MID)
                        ),
                        new CommandGroupBaseAction(new SequentialCommandGroup(
                                new InstantCommand(outtakeSusystem::go_outtake_first, outtakeSusystem),
                                new WaitCommand(80),
                                new InstantCommand(outtakeSusystem::go_outtake_second, outtakeSusystem)
                        )),
                        RR.RobotToBackdrop().build()
                ),

                new CommandGroupBaseAction(new SequentialCommandGroup(
                        new InstantCommand(outtakeSusystem::wheel_release, outtakeSusystem),
                        new WaitCommand(2000),
                        new InstantCommand(outtakeSusystem::wheel_stop, outtakeSusystem)
                )),
                new ParallelAction(
                        new CommandGroupBaseAction(
                                new SequentialCommandGroup(
                                        new InstantCommand(outtakeSusystem::go_intake_second, outtakeSusystem),
                                        new WaitCommand(80),
                                        new InstantCommand(outtakeSusystem::go_intake_first, outtakeSusystem)
                                )
                        ),
                        new CommandAction(new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.LOADING))
                ),
                RR.RobotBackdropToStation().build(),
                RR.RobotStation().first.build(),
                RR.RobotStation().second.build(),
                RR.RobotStationToBackdrop().build(),
                RR.RobotParking().build()
        ));
//        Actions.runBlocking(RR.test);
    }
}
