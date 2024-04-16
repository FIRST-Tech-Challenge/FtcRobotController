package org.firstinspires.ftc.masters.world;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.masters.PropFindRightProcessor;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;
import org.firstinspires.ftc.masters.world.paths.BlueBackDropPath;

@Config
@Autonomous(name = "Blue Backdrop 2 + 0", group = "competition")
public class BlueBackDrop_2_0 extends BackDropOpMode {


    Vector2d yellowLeftPos = new Vector2d();
    Vector2d yellowMidPos = new Vector2d();
    Vector2d yellowRightPos = new Vector2d();

    Pose2d tagAlignmentPosition = new Pose2d(54, 36, Math.toRadians(180));

    @Override
    protected void initializeProp(){
        drive.initializePropFindLeftProcessing();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initAuto();

        Pose2d startPose = new Pose2d(new Vector2d(16, 61.2), Math.toRadians(270)); //Start position for roadrunner
        drive.setPoseEstimate(startPose);
        drive.setWristServoPosition(outtakeWristPosition);

        //PURPLE PIXEL

       rightPurple = BlueBackDropPath.getRightPurple(drive, startPose);


        leftPurple = BlueBackDropPath.getLeftPurple(drive, startPose);

        middlePurple = BlueBackDropPath.getMidPurple(drive, startPose);


        TrajectorySequence tagAlignLeft = drive.trajectorySequenceBuilder(leftPurple.end())
                .back(5)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(tagAlignmentPosition, Math.toRadians(0))
                .build();

        TrajectorySequence tagAlignMid = drive.trajectorySequenceBuilder(middlePurple.end())
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(tagAlignmentPosition, Math.toRadians(0))
                .build();

        TrajectorySequence tagAlignRight = drive.trajectorySequenceBuilder(rightPurple.end())
                .back(5)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(tagAlignmentPosition, Math.toRadians(0))
                .build();


        //YELLOW PIXELS

        leftYellow = BlueBackDropPath.getLeftYellow(drive, leftPurple.end());
        midYellow = BlueBackDropPath.getMidYellow(drive, middlePurple.end());
        rightYellow = BlueBackDropPath.getRightYellow(drive, rightPurple.end());

        parkFromLeft = BlueBackDropPath.park(drive, leftYellow.end());
        parkFromRight =BlueBackDropPath.park(drive, rightYellow.end());
        parkFromMid = BlueBackDropPath.park(drive, midYellow.end());


        //OTHER PATHS
//
//        TrajectorySequence backAway = drive.trajectorySequenceBuilder(rightYellow.end())
//                .forward(5)
//
//                .build();
//
//
//        TrajectorySequence Park = drive.trajectorySequenceBuilder(backAway.end())
//                .setTangent(Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(48, 58, Math.toRadians(180)), Math.toRadians(0))
//
//                .build();



        drive.raiseIntake();
        drive.closeFingers();


        propPos = drive.getPropFindProcessor().position;

        waitForStart();

        currentState = State.PURPLE_DEPOSIT_PATH;
        drive.dropIntake();

        retrievePropPos();


//TO DO: go park

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.update();
            drive.update();
            drive.backSlidesMove(outtakeTarget);

            switch (currentState){
                case PURPLE_DEPOSIT_PATH:
                    purpleDepositPathState();
                    break;
                case PURPLE_DEPOSIT:
                    purpleDepositState();
                    break;
                case BACKDROP_DEPOSIT_PATH:
                    if (propPos== PropFindRightProcessor.pos.LEFT){
                        backdropDepositPath(State.PARK, parkFromLeft);
                    } else if (propPos== PropFindRightProcessor.pos.RIGHT){
                        backdropDepositPath(State.PARK, parkFromRight);
                    } else if (propPos== PropFindRightProcessor.pos.MID) {
                        backdropDepositPath(State.PARK, parkFromMid);
                    }
                    break;
                case PARK:
                    park();
                    break;

            }



        }
    }

    public TrajectorySequence getStackWingTrajectory(Pose2d robotPosition){
        return BlueBackDropPath.toStackWing(drive, robotPosition);
    }
}