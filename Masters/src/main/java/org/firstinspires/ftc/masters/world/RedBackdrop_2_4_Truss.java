package org.firstinspires.ftc.masters.world;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.masters.PropFindRightProcessor;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;
import org.firstinspires.ftc.masters.world.paths.RedBackDropPath;

@Config
@Autonomous(name = "Red Backdrop 2 + 4 Truss", group = "competition")
public class RedBackdrop_2_4_Truss extends BackDropOpMode {
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

        rightPurple = RedBackDropPath.getRightPurple(drive, startPose);


        leftPurple = RedBackDropPath.getLeftPurple(drive, startPose);

        middlePurple = RedBackDropPath.getMidPurple(drive, startPose);


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

        leftYellow = RedBackDropPath.getLeftYellow(drive, leftPurple.end());


        midYellow = RedBackDropPath.getMidYellow(drive, middlePurple.end());


        rightYellow = RedBackDropPath.getRightYellow(drive, rightPurple.end());


        //OTHER PATHS

        toStackFromMid= RedBackDropPath.toStackTruss(drive, midYellow.end());
        toStackFromRight= RedBackDropPath.toStackTruss(drive, rightYellow.end());
        toStackFromLeft = RedBackDropPath.toStackTruss(drive, leftYellow.end());

        toBackBoard = RedBackDropPath.fromStackToBoardTruss(drive, toStackFromMid.end());

        park = RedBackDropPath.park(drive, toBackBoard.end());




        drive.raiseIntake();
        drive.closeFingers();


        propPos = drive.getPropFindProcessor().position;

        waitForStart();

        currentState = State.PURPLE_DEPOSIT_PATH;
        drive.dropIntake();

        retrievePropPos();


        TrajectorySequence nextPath= null;

//TO DO: go park

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            drive.backSlidesMove(outtakeTarget);

            switch (currentState) {
                case PURPLE_DEPOSIT_PATH:
                    purpleDepositPathState();
                    break;
                case PURPLE_DEPOSIT:
                    purpleDepositState();
                    break;
                case BACKDROP_DEPOSIT_PATH:
                    if (cycleCount <=1){
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

                case PARK:
                    park();
                    break;

            }



        }
    }
}