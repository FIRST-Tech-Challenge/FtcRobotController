package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.CenterStageRobot.commands.ElevatorCommand;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeArmSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.OuttakeSusystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.PixelFingerSubsystem;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequenceBuilder;

public class RoadRunnerSubsystem_BLUE {
    protected SampleMecanumDrive drive;
    /*-------------------------------------------------------
    -Mechanisms-
    -------------------------------------------------------*/
    protected OuttakeSusystem outtakeSusystem;
    protected ElevatorSubsystem elevatorSubsystem;
    protected ElevatorCommand elevatorCommand;
    protected IntakeArmSubsystem intakeArmSubsystem;
    protected IntakeSubsystem intakeSubsystem;
    protected PixelFingerSubsystem pixelFingerSubsystem;
    /*-------------------------------------------------------
    -Params-
    -------------------------------------------------------*/
    public final double Tile = 24; /*-inches-*/
    public final double TileInverted = -24; /*-inches-*/
    public final double RobotX = 12.5984252; /*-inches-*/
    public final double RobotY = 16.9291339; /*-inches-*/
    public final double BackdropDistance = 0; /*-inches-*/
    /*-------------------------------------------------------
    -Trajectories-
    -------------------------------------------------------*/
    protected TrajectorySequenceBuilder test;

    protected TrajectorySequenceBuilder leftSpike;
    protected TrajectorySequenceBuilder centerSpike;
    protected TrajectorySequenceBuilder rightSpike;

    protected TrajectorySequenceBuilder pixel_backdrop_Short;
    protected TrajectorySequenceBuilder pixel_backdrop_Long;

    protected TrajectorySequenceBuilder parking;
    /*-------------------------------------------------------
    -Enums-
    -------------------------------------------------------*/
    enum Randomization{
        LEFT,
        CENTER,
        RIGHT
    }

    enum StartingPosition{
        SHORT,
        LONG
    }

    enum Path{
        INNER,
        OUTER
    }

    enum ParkingPosition{
        INNER,
        MID,
        OUTER
    }

    enum PixelStack{
        INNER,
        MID,
        OUTER
    }

    protected StartingPosition startingPosition;
    protected Path path;
    protected ParkingPosition parkingPosition;
    protected PixelStack pixelStack;
    /*-------------------------------------------------------
    -Poses-
    -------------------------------------------------------*/

    protected Pose2d leftPixelSpike;
    protected Pose2d centerPixelSpike;
    protected Pose2d rightPixelSpike;
    protected Pose2d randomizedBackdrop;
    protected Vector2d stationClose;
    protected Vector2d stationFar;
    protected Pose2d backdrop_Unload;
    protected Pose2d stackStation;
    protected Pose2d parkingPose;

    protected Integer[] leftSpikeStartingTanget = {45, 135}; //For short 45 and long 135 difference
    protected Integer[] leftSpikeFinalTanget = {180, 0}; //For short 180 and long 0 difference
    protected Integer[] stackStationTanget = {180, 225, 135}; // 180 for Inner 225 for Mid and Outer FROM INNER// 135 for FROM OUTER
    protected Integer[] parkingTanget = {135, 225}; // 135 for Inner 225 for Mid and Outer

    protected Integer leftSpikeStartingTangetValue = 0;
    protected Integer leftSpikeFinalTangetValue = 0;
    protected Integer stackStationTangetValue = 0;
    protected Integer parkingTangetValue = 1;

    protected Pose2d leftPixel_SHORT = new Pose2d((RobotY/2), Tile + (RobotX/2), Math.toRadians(0));
    protected Pose2d centerPixel_SHORT = new Pose2d(Tile/2, Tile + (RobotY/2), Math.toRadians(270));
    protected Pose2d rightPixel_SHORT = new Pose2d(Tile, 1.5 * Tile, Math.toRadians(180));

    protected Pose2d leftPixel_LONG = new Pose2d(2 * TileInverted,1.5 * Tile, Math.toRadians(180));
    protected Pose2d centerPixel_LONG = new Pose2d(1.5 * TileInverted, Tile + (RobotY/2), Math.toRadians(270));
    protected Pose2d rightPixel_LONG = new Pose2d(TileInverted - (RobotY/2), Tile + (RobotX/2), Math.toRadians(0));

    protected Pose2d backdropLeft = new Pose2d(2.5 * Tile - (RobotY/2), 1.25 * Tile, Math.toRadians(180)); // Default
    protected Pose2d backdropCenter = new Pose2d(2.5 * Tile - (RobotY/2), 1.5 * Tile, Math.toRadians(180));
    protected Pose2d backdropRight = new Pose2d(2.5 * Tile - (RobotY/2), 1.75 * Tile, Math.toRadians(180)); // Default

    protected Pose2d stationInner = new Pose2d(3 * TileInverted + (RobotY/2),Tile/2, Math.toRadians(180)); // Default
    protected Pose2d stationMiddle = new Pose2d(3 * TileInverted + (RobotY/2),Tile, Math.toRadians(180));
    protected Pose2d stationOuter = new Pose2d(3 * TileInverted + (RobotY/2), 1.5 * Tile, Math.toRadians(180)); // Default

    protected Pose2d parkingInner = new Pose2d(2.5 * Tile, Tile/2, Math.toRadians(180));
    protected Pose2d parkingMiddle = new Pose2d(2 * Tile, 1.5 * Tile, Math.toRadians(180));
    protected Pose2d parkingOuter = new Pose2d(2.5 * Tile, 2.5 * Tile, Math.toRadians(180));

    protected Vector2d stationClose_Inner = new Vector2d(Tile, Tile/2);
    protected Vector2d stationFar_Inner = new Vector2d(2 * TileInverted,Tile/2);

    protected Vector2d stationClose_Outer = new Vector2d(Tile, 2.5 * Tile);
    protected Vector2d stationFar_Outer = new Vector2d(2 * TileInverted,2.5 * Tile);

    protected Pose2d pixel_cycle_PoseTransfer = centerPixel_SHORT;
    /*-------------------------------------------------------
    -FTCLib Commands-
    -------------------------------------------------------*/

    public SequentialCommandGroup randomizationPixelElevator(){
        return new SequentialCommandGroup(
                new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.AUTO),
                new InstantCommand(outtakeSusystem::go_outtake_first, outtakeSusystem),
                new WaitCommand(80),
                new InstantCommand(outtakeSusystem::go_outtake_second, outtakeSusystem)
        );
    }

    public SequentialCommandGroup elevator(){
        return new SequentialCommandGroup(
                new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.LOW),
                new InstantCommand(outtakeSusystem::go_outtake_first, outtakeSusystem),
                new WaitCommand(80),
                new InstantCommand(outtakeSusystem::go_outtake_second, outtakeSusystem)
        );
    }

    public SequentialCommandGroup scoring(){
        return new SequentialCommandGroup(
                new SequentialCommandGroup(
                        new InstantCommand(outtakeSusystem::wheel_release, outtakeSusystem),
                        new WaitCommand(2000),
                        new InstantCommand(outtakeSusystem::wheel_stop, outtakeSusystem)),

                new SequentialCommandGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(outtakeSusystem::go_intake_second, outtakeSusystem),
                                new WaitCommand(80),
                                new InstantCommand(outtakeSusystem::go_intake_first, outtakeSusystem)
                        ),
                        new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.LOADING)
                )
        );
    }

    public SequentialCommandGroup stackStationIntake() {
        return new SequentialCommandGroup(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new InstantCommand(intakeSubsystem::run_auto, intakeSubsystem),
                                new InstantCommand(outtakeSusystem::wheel_grab)
                        ),
                        new WaitCommand(1000)
                ),
                new SequentialCommandGroup(
                        new InstantCommand(outtakeSusystem::wheel_stop),
                        new InstantCommand(intakeArmSubsystem::raiseArm),
                        new InstantCommand(intakeSubsystem::reverse),
                        new WaitCommand(1000),
                        new InstantCommand(intakeSubsystem::stop, intakeSubsystem)
                )
        );
    }

    /*-------------------------------------------------------
    -La program-
    -------------------------------------------------------*/
    RoadRunnerSubsystem_BLUE(HardwareMap hardwareMap, Pose2d HomePose,
                             StartingPosition startingPosition, Path path, PixelStack pixelStack,
                             ParkingPosition parkingPosition){

        this.drive = new SampleMecanumDrive(hardwareMap);
        this.startingPosition = startingPosition;
        this.path = path;
        this.pixelStack = pixelStack;
        this.parkingPosition = parkingPosition;

        /*-----------------------------------------------------*/

        outtakeSusystem = new OuttakeSusystem(hardwareMap);
        elevatorSubsystem = new ElevatorSubsystem(hardwareMap, telemetry, () -> 0);
        intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        intakeArmSubsystem = new IntakeArmSubsystem(hardwareMap);
        pixelFingerSubsystem = new PixelFingerSubsystem(hardwareMap);

        /*-----------------------------------------------------*/

        drive.setPoseEstimate(HomePose);

        /*-----------------------------------------------------*/
        test = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeTo(new Vector2d(0,0));
        /*-----------------------------------------------------*/

        leftSpike = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setTangent(Math.toRadians(leftSpikeStartingTanget[leftSpikeStartingTangetValue])) //tan pair 45/135
                .splineToLinearHeading(leftPixelSpike, Math.toRadians(leftSpikeFinalTanget[leftSpikeFinalTangetValue]))
                .addDisplacementMarker(() -> {
                    new InstantCommand(pixelFingerSubsystem::release, pixelFingerSubsystem);
                });//tan pair 180/0

        /*------------------------------------------------------------------------*/

        centerSpike = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineTo(centerPixelSpike.vec())
                .addDisplacementMarker(() -> {
                    new InstantCommand(pixelFingerSubsystem::release, pixelFingerSubsystem);
                });

        /*------------------------------------------------------------------------*/

        rightSpike = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .strafeTo(rightPixelSpike.vec())
                .addDisplacementMarker(() -> {
                    new InstantCommand(pixelFingerSubsystem::release, pixelFingerSubsystem);
                });

        /*----------------------------------------------------------------------------------------*/

        pixel_backdrop_Short = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addDisplacementMarker(() -> {
                    randomizationPixelElevator();
                })
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(randomizedBackdrop, Math.toRadians(300))
                .addDisplacementMarker(() -> {
                    scoring();
                })
                /*-------------------------------------------------------------------*/
                /*----2+2----*/
                /*-------------------------------------------------------------------*/
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(stationClose, Math.toRadians(180))
                .lineTo(stationFar)
                .splineToConstantHeading(stackStation.vec(), Math.toRadians(stackStationTanget[stackStationTangetValue])) //tan pair 180/225
                .addDisplacementMarker(() -> {
                    stackStationIntake();
                })
                .setReversed(true)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(stationFar, Math.toRadians(0))
                .lineTo(stationClose)
                .addDisplacementMarker(() -> {
                    elevator();
                })
                .splineToConstantHeading(backdrop_Unload.vec(), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    scoring();
                })
                /*-------------------------------------------------------------------*/
                /*----2+4----*/
                /*-------------------------------------------------------------------*/
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(stationClose, Math.toRadians(180))
                .lineTo(stationFar)
                .splineToConstantHeading(stackStation.vec(), Math.toRadians(stackStationTanget[stackStationTangetValue])) //tan pair 180/225
                .addDisplacementMarker(() -> {
                    stackStationIntake();
                })
                .setReversed(true)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(stationFar, Math.toRadians(0))
                .lineTo(stationClose)
                .addDisplacementMarker(() -> {
                    elevator();
                })
                .splineToConstantHeading(backdrop_Unload.vec(), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    scoring();
                });
        /*----------------------------------------------------------------------------------------*/

        pixel_backdrop_Long = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(stackStation, Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    stackStationIntake();
                })
                .setReversed(true)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(stationFar, Math.toRadians(0))
                .lineTo(stationClose)
                .addDisplacementMarker(() -> {
                    randomizationPixelElevator();
                })
                .splineToConstantHeading(randomizedBackdrop.vec(), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    scoring();
                })
                /*-------------------------------------------------------------------*/
                /*----2+3----*/
                /*-------------------------------------------------------------------*/
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(stationClose, Math.toRadians(180))
                .lineTo(stationFar)
                .splineToConstantHeading(stackStation.vec(), Math.toRadians(stackStationTanget[stackStationTangetValue])) //tan pair 180/225
                .addDisplacementMarker(() -> {
                    stackStationIntake();
                })
                .setReversed(true)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(stationFar, Math.toRadians(0))
                .lineTo(stationClose)
                .addDisplacementMarker(() -> {
                    elevator();
                })
                .splineToConstantHeading(backdrop_Unload.vec(), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    scoring();
                })
                /*-------------------------------------------------------------------*/
                /*----2+5----*/
                /*-------------------------------------------------------------------*/
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(stationClose, Math.toRadians(180))
                .lineTo(stationFar)
                .splineToConstantHeading(stackStation.vec(), Math.toRadians(stackStationTanget[stackStationTangetValue])) //tan pair 180/225
                .addDisplacementMarker(() -> {
                    stackStationIntake();
                })
                .setReversed(true)
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(stationFar, Math.toRadians(0))
                .lineTo(stationClose)
                .addDisplacementMarker(() -> {
                    elevator();
                })
                .splineToConstantHeading(backdrop_Unload.vec(), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    scoring();
                });

        /*----------------------------------------------------------------------------------------*/

        parking = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .setTangent(Math.toRadians(parkingTanget[parkingTangetValue])) //tan 135/225
                .splineToConstantHeading(parkingPose.vec(), Math.toRadians(0));
    }
}