package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "CenterStageAutonomous_Inverted", group = "Final Autonomous")
public class CenterStageAutonomous_Inverted extends LinearOpMode {

    protected MecanumDrive drive;
    protected RoadRunnerSubsystem RR;

    protected Pose2d homePose = new Pose2d((3 * RR.Tile) - (RR.RobotY/2),(RR.TileInverted/2),Math.toRadians(0));

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, homePose);
        RR = new RoadRunnerSubsystem(drive, RoadRunnerSubsystem.Alliance.BLUE, RoadRunnerSubsystem.Start.LOW,
                RoadRunnerSubsystem.Corridor.INNER, RoadRunnerSubsystem.Corridor.INNER,
                RoadRunnerSubsystem.Station.INNER, RoadRunnerSubsystem.Parking.INNER);

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                RR.LOW_HomeToPixel_LEFT.build(),
                RR.RobotToBackdrop().build(),
                RR.RobotBackdropToStation().build(),
                RR.RobotStation().first.build(),
                RR.RobotStation().second.build(),
                RR.RobotStationToBackdrop().build(),
                RR.RobotParking().build()
        ));
//        Actions.runBlocking(RR.test);
    }
}

