package org.firstinspires.ftc.masters.world;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.masters.PropFindRightProcessor;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;
import org.firstinspires.ftc.masters.world.paths.RedFarSidePath;

@Config
@Autonomous(name = "Red Far Side 2 + 1", group = "competition")
public class RedFarSide_2_1 extends FarSideOpMode {


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


        Pose2d startPose = new Pose2d(new Vector2d(-39, -61.2), Math.toRadians(90)); //Start position for roadrunner
        drive.setPoseEstimate(startPose);
        drive.setWristServoPosition(outtakeWristPosition);

        //PURPLE PIXEL

        rightPurple = RedFarSidePath.getRightPurple(drive, startPose);

        leftPurple = RedFarSidePath.getLeftPurple(drive, startPose);

        middlePurple = RedFarSidePath.getMidPurple(drive, startPose);

        rightPurpleToStack = RedFarSidePath.getRightPurpleToStack(drive, rightPurple.end());


        leftPurpleToStack = RedFarSidePath.getLeftPurpleToStack(drive, leftPurple.end());

        midPurpleToStack = RedFarSidePath.getMidPurpleToStack(drive, middlePurple.end());

        stackToRightYellow = RedFarSidePath.getStackToRightYellow(drive, rightPurpleToStack.end());

        stackToMidYellow = RedFarSidePath.getStackToMidYellow(drive, midPurpleToStack.end());

        stackToLeftYellow = RedFarSidePath.getStackToLeftYellow(drive, leftPurpleToStack.end());




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



        //OTHER PATHS

        TrajectorySequence backAway = drive.trajectorySequenceBuilder(stackToRightYellow.end())
                .forward(5)

                .build();


        TrajectorySequence Park = drive.trajectorySequenceBuilder(backAway.end())
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(48, 58, Math.toRadians(180)), Math.toRadians(0))

                .build();



        propPos = drive.getPropFindProcessor().position;

        waitForStart();

        currentState = State.PURPLE_DEPOSIT_PATH;

        drive.dropIntake();

        retrievePropPos();

        TrajectorySequence nextPath= null;

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            drive.backSlidesMove(outtakeTarget);


            switch (currentState){
                case PURPLE_DEPOSIT_PATH:
                    purpleDepositPath();
                    break;
                case PURPLE_DEPOSIT:
                    purpleDeposit();
                    break;
                case TO_STACK:
                    switch (propPos){
                        case LEFT:
                            nextPath = stackToLeftYellow;
                            break;
                        case RIGHT:
                            nextPath = stackToRightYellow;
                            break;
                        case MID:
                            nextPath = stackToMidYellow;
                            break;
                    }
                    toStack(nextPath);
                    break;
                case BACKDROP_DEPOSIT_PATH:
                    backdropDepositPath(State.PARK, park);
                    break;

            }


        }
    }
}