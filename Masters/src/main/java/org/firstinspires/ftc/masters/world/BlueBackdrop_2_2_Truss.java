package org.firstinspires.ftc.masters.world;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.masters.PropFindRightProcessor;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;
import org.firstinspires.ftc.masters.world.paths.BlueBackDropPath;

@Config
@Autonomous(name = "Blue backdrop 2 + 2 Truss", group = "competition")
public class BlueBackdrop_2_2_Truss extends BackDropOpMode {


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

        //cycle

        toStackFromMid= BlueBackDropPath.toStackTruss(drive, midYellow.end());
        toStackFromRight= BlueBackDropPath.toStackTruss(drive, rightYellow.end());
        toStackFromLeft = BlueBackDropPath.toStackTruss(drive, leftYellow.end());

        toBackBoard = BlueBackDropPath.fromStackToBoardTruss(drive, toStackFromMid.end());

        park = BlueBackDropPath.park(drive, toBackBoard.end());



        //OTHER PATHS

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

        currentState = BackDropOpMode.State.PURPLE_DEPOSIT_PATH;
        drive.dropIntake();

        retrievePropPos();


        TrajectorySequence nextPath=null;

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
                    if (cycleCount == 0){
                        switch (propPos) {
                            case RIGHT:
                                nextPath = toStackFromRight;
                                break;
                            case LEFT:
                                nextPath = toStackFromLeft;
                                break;
                            case MID:
                                drive.followTrajectorySequenceAsync(toStackFromMid);
                                break;
                        }
                        backdropDepositPath(State.TO_STACK, nextPath);

                    } else {
                        backdropDepositPath(State.PARK , park);
                    }

                    break;
                case TO_STACK:
                    toStack();
                    break;
                case TO_STACK_TAG:
                    toStackTag();
                    break;
                case PARK:
                    park();
                    break;

            }



        }
    }
}