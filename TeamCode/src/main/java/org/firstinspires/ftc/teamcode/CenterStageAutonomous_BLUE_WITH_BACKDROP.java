package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CenterStageRobot.commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeArmSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.OuttakeSusystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.PixelFingerSubsystem;
import org.firstinspires.ftc.teamcode.Extra.CommandAction;
import org.firstinspires.ftc.teamcode.Extra.CommandGroupBaseAction;
import org.inventors.ftc.opencvpipelines.TeamPropDetectionPipeline;
import org.inventors.ftc.robotbase.hardware.Camera;

@Autonomous(name = "CenterStageAutonomous_BLUE_WITH_BACKDROP", group = "Final Autonomous")
public class CenterStageAutonomous_BLUE_WITH_BACKDROP extends LinearOpMode {

    protected MecanumDrive drive;
    protected RoadRunnerSubsystem_BLUE RR;
    protected OuttakeSusystem outtakeSusystem;
    protected ElevatorSubsystem elevatorSubsystem;
    protected ElevatorCommand elevatorCommand;
    protected IntakeArmSubsystem intakeArmSubsystem;
    protected IntakeSubsystem intakeSubsystem;
    protected PixelFingerSubsystem pixelFingerSubsystem;

    public RoadRunnerSubsystem_BLUE.Randomizer randomizer;
    public TrajectoryActionBuilder ToPixel, ToBackdrop, ToMid;

    protected static double starting_pos_error_X = 0;//inch
    protected static double starting_pos_error_Y = 0;//inch
    protected static RoadRunnerSubsystem_BLUE.Alliance alliance = RoadRunnerSubsystem_BLUE.Alliance.RED;
    protected Pose2d homePose_LOW_RED = new Pose2d((3 * RR.TileInverted) + (RR.RobotY/2) + starting_pos_error_X,0 - (RR.RobotX/2),Math.toRadians(180));
    protected Pose2d homePose_HIGH_RED = new Pose2d((3 * RR.TileInverted) + (RR.RobotY/2),(RR.Tile * 1.5),Math.toRadians(180));
    protected Pose2d homePose_HIGH_BLUE = new Pose2d(3 * RoadRunnerSubsystem_BLUE.Tile - (RoadRunnerSubsystem_BLUE.RobotY/2) - starting_pos_error_X, RoadRunnerSubsystem_BLUE.Tile * 1.5, Math.toRadians(0));
    protected Pose2d homePose_LOW_BLUE = new Pose2d(3 * RoadRunnerSubsystem_BLUE.Tile - (RoadRunnerSubsystem_BLUE.RobotY/2) - starting_pos_error_X, RoadRunnerSubsystem_BLUE.TileInverted/2, Math.toRadians(0));
    protected Camera camera;
    protected FtcDashboard dashboard;
    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, homePose_LOW_RED);
        RR = new RoadRunnerSubsystem_BLUE(drive, alliance, RoadRunnerSubsystem_BLUE.Start.LOW,
                RoadRunnerSubsystem_BLUE.Corridor.INNER, RoadRunnerSubsystem_BLUE.Corridor.INNER,
                RoadRunnerSubsystem_BLUE.Station.INNER, RoadRunnerSubsystem_BLUE.Parking.OUTER);

        ////////////////////////////////////////////////////////////////////////////////////////////

        outtakeSusystem = new OuttakeSusystem(hardwareMap);
        elevatorSubsystem = new ElevatorSubsystem(hardwareMap, telemetry, () -> 0);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        intakeArmSubsystem = new IntakeArmSubsystem(hardwareMap);
        pixelFingerSubsystem = new PixelFingerSubsystem(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        camera = new Camera(hardwareMap, dashboard, telemetry, TeamPropDetectionPipeline.Alliance.BLUE);

        ////////////////////////////////////////////////////////////////////////////////////////////

        waitForStart();

        ////////////////////////////////////////////////////////////////////////////////////////////

        randomizer = RoadRunnerSubsystem_BLUE.Randomizer.values()[camera.getTeamPropPos()];

        if (randomizer == RoadRunnerSubsystem_BLUE.Randomizer.LEFT){
            ToMid = RR.LOW_ToMid_LEFT;
            ToBackdrop = RR.LOW_ToBackdrop_MID.strafeTo(new Vector2d(1.75 * RR.TileInverted, RR.Tile - (RR.RobotY/2) + RR.BackdropDistance_FIRST));
        }else if (randomizer == RoadRunnerSubsystem_BLUE.Randomizer.CENTER){
            ToMid = RR.LOW_ToMid_CENTER;
            ToBackdrop = RR.LOW_ToBackdrop_MID;
        }else if (randomizer == RoadRunnerSubsystem_BLUE.Randomizer.RIGHT){
            ToMid = RR.LOW_ToMid_RIGHT;
            ToBackdrop = RR.LOW_ToBackdrop_MID.strafeTo(new Vector2d(1.25 * RR.TileInverted, RR.Tile - (RR.RobotY/2) + RR.BackdropDistance_FIRST));
        }

        telemetry.addData("Team Prop Position", camera.getTeamPropPos());
        telemetry.addData("Randomizer", randomizer);
        telemetry.update();

        ToPixel = RR.RobotToBackdrop(randomizer).first;

        ////////////////////////////////////////////////////////////////////////////////////////////

        Actions.runBlocking(new SequentialAction(

                ////////////////////////////////////////////////////////////////////////////////////
                ToPixel.build(),
                new CommandAction(new InstantCommand(pixelFingerSubsystem::release)),
                new CommandAction(new WaitCommand(2000)),
                ToMid.build(),

                new ParallelAction(
                        new CommandAction(
                                new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.AUTO)
                        ),
                        new CommandGroupBaseAction(new SequentialCommandGroup(
                                new InstantCommand(outtakeSusystem::go_outtake_first, outtakeSusystem),
                                new WaitCommand(80),
                                new InstantCommand(outtakeSusystem::go_outtake_second, outtakeSusystem)
                        )),

                        ////////////////////////////////////////////////////////////////////////////////////
                        ToBackdrop.build()
                        ////////////////////////////////////////////////////////////////////////////////////

                ),

                new CommandGroupBaseAction(new SequentialCommandGroup(
                        new InstantCommand(outtakeSusystem::wheel_release, outtakeSusystem),
                        new WaitCommand(2000),
                        new InstantCommand(outtakeSusystem::wheel_stop, outtakeSusystem)
                )),

                new ParallelAction(
                        new CommandGroupBaseAction(
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
//                                        new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new InstantCommand(outtakeSusystem::go_intake_second, outtakeSusystem),
                                                new WaitCommand(80),
                                                new InstantCommand(outtakeSusystem::go_intake_first, outtakeSusystem)
                                        ),
                                        new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.LOADING)
//                                        )
                                )
                        ),

                        RR.LOW_ToBackdrop_OUTER.build()
                )




//                ////////////////////////////////////////////////////////////////////////////////////
//
//                new ParallelAction(
//                        new CommandAction(
//                                new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.AUTO)
//                        ),
//                        new CommandGroupBaseAction(new SequentialCommandGroup(
//                                new InstantCommand(outtakeSusystem::go_outtake_first, outtakeSusystem),
//                                new WaitCommand(80),
//                                new InstantCommand(outtakeSusystem::go_outtake_second, outtakeSusystem)
//                        )),
//
//                ////////////////////////////////////////////////////////////////////////////////////
//                        ToBackdrop.build()
//                ////////////////////////////////////////////////////////////////////////////////////
//
//                ),
//
//                new CommandGroupBaseAction(new SequentialCommandGroup(
//                        new InstantCommand(outtakeSusystem::wheel_release, outtakeSusystem),
//                        new WaitCommand(2000),
//                        new InstantCommand(outtakeSusystem::wheel_stop, outtakeSusystem)
//
//                )),
//
//                new ParallelAction(
//
//                ////////////////////////////////////////////////////////////////////////////////////
//                        RR.RobotParking().build(),
//                ////////////////////////////////////////////////////////////////////////////////////
//
//                        new CommandGroupBaseAction(
//                                new SequentialCommandGroup(
//                                        new InstantCommand(outtakeSusystem::go_intake_second, outtakeSusystem),
//                                        new WaitCommand(80),
//                                        new InstantCommand(outtakeSusystem::go_intake_first, outtakeSusystem)
//                                )
//                        ),
//                        new CommandAction(new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.LOADING))
//                )
        ));
//        Actions.runBlocking(RR.test);
    }
}
