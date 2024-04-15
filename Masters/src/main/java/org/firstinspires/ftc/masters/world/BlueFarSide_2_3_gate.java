package org.firstinspires.ftc.masters.world;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.masters.PropFindRightProcessor;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;
import org.firstinspires.ftc.masters.world.paths.BlueFarSidePath;

@Config
@Autonomous(name = "Far Side blue 2 + 3 Gate", group = "competition")
public class BlueFarSide_2_3_gate extends FarSideOpMode {


    Vector2d yellowLeftPos = new Vector2d();
    Vector2d yellowMidPos = new Vector2d();
    Vector2d yellowRightPos = new Vector2d();

    Pose2d tagAlignmentPosition = new Pose2d(54, 36, Math.toRadians(180));

    @Override
    protected void initializeProp(){
        drive.initializePropFindRightProcessing();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initAuto();


        Pose2d startPose = new Pose2d(new Vector2d(-39, 61.2), Math.toRadians(270)); //Start position for roadrunner
        drive.setPoseEstimate(startPose);
        drive.setWristServoPosition(outtakeWristPosition);

        //PURPLE PIXEL

        rightPurple = BlueFarSidePath.getRightPurple(drive, startPose);

        leftPurple = BlueFarSidePath.getLeftPurple(drive, startPose);

        middlePurple = BlueFarSidePath.getMidPurple(drive, startPose);

        rightPurpleToStack = BlueFarSidePath.getRightPurpleToStack(drive, rightPurple.end());


        leftPurpleToStack = BlueFarSidePath.getLeftPurpleToStack(drive, leftPurple.end());

        midPurpleToStack = BlueFarSidePath.getMidPurpleToStack(drive, middlePurple.end());

        stackToRightYellow = BlueFarSidePath.getStackToRightYellow(drive, rightPurpleToStack.end());

        stackToMidYellow = BlueFarSidePath.getStackToMidYellow(drive, midPurpleToStack.end());

        stackToLeftYellow = BlueFarSidePath.getStackToLeftYellow(drive, leftPurpleToStack.end());

        toStackCycleGateLeft = BlueFarSidePath.toStackFromBackboardGate(drive, stackToLeftYellow.end() );
        toStackCycleGateMid = BlueFarSidePath.toStackFromBackboardGate(drive, stackToMidYellow.end());
        toStackCycleGateRight = BlueFarSidePath.toStackFromBackboardGate(drive, stackToRightYellow.end());

        toBackboardCycleGate = BlueFarSidePath.toBackboardGate(drive, toStackCycleGateLeft.end());



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

        propPos= PropFindRightProcessor.pos.RIGHT;
        TrajectorySequence nextPath=null;

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
                    if (cycleCount ==0) {
                        switch (propPos) {
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
                    } else {
                        nextPath= toBackboardCycleGate;
                    }

                    toStack(nextPath);
                    break;
                case BACKDROP_DEPOSIT_PATH:
                    if (cycleCount==0) {
                        switch (propPos) {
                            case LEFT:
                                nextPath = toStackCycleGateLeft;
                                break;
                            case RIGHT:
                                nextPath = toStackCycleGateRight;
                                break;
                            case MID:
                                nextPath = toStackCycleGateMid;
                                break;
                        }
                    } else {
                        backdropDepositPath(State.PARK, park);
                    }
                    break;
                case PARK:
                    park();
                    break;

            }


        }
    }
}