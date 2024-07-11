package org.firstinspires.ftc.masters.world;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.masters.CSCons;
import org.firstinspires.ftc.masters.PropFindRightProcessor;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;
import org.firstinspires.ftc.masters.world.paths.RedFarSidePath;

@Config
@Autonomous(name = "Red Far Side 2 + 3", group = "competition")
public class RedFarSide_2_3 extends FarSideOpMode {


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

        parkFromMid = RedFarSidePath.park(drive, stackToMidYellow.end());
        parkFromLeft = RedFarSidePath.park(drive, stackToLeftYellow.end());
        parkFromRight = RedFarSidePath.park(drive, stackToRightYellow.end());

        toStackCycleGateLeft = RedFarSidePath.toStackCenterFromBackboardGate(drive, stackToLeftYellow.end() );
        toStackCycleGateMid = RedFarSidePath.toStackCenterFromBackboardGate(drive, stackToMidYellow.end());
        toStackCycleGateRight = RedFarSidePath.toStackCenterFromBackboardGate(drive, stackToRightYellow.end());

        toStackFromCenterGate = RedFarSidePath.toStack1FromBackboardGate(drive, toStackCycleGateLeft.end());

        toBackboardCycleGate = RedFarSidePath.toBackboardGate(drive, toStackCycleGateLeft.end());



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



        propPos = drive.getPropFindProcessor().position;

        waitForStart();

        currentState = State.PURPLE_DEPOSIT_PATH;

        retrievePropPos();

        TrajectorySequence nextPath= null;

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            drive.backSlidesMove(outtakeTarget);
            telemetry.addData("cycle", cycleCount);
            telemetry.addData("target", outtakeTarget);
            telemetry.addData("state", currentState);

            telemetry.update();

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
                        stackToLeftYellow = RedFarSidePath.getStackToLeftYellow(drive,toStackCycleGateLeft.end());
                        nextPath= stackToLeftYellow;
                    }

                    toStack(nextPath);
                    break;
                case TO_STACK_1:
                    toStack1();
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
                        backdropDepositPath(State.TO_STACK_1, nextPath);

                    } else {
                        backdropDepositPath(State.PARK, parkFromLeft);
                    }
                    break;
                case PARK:
                    park();
                    break;

            }


        }
    }

    @Override
    public CSCons.OuttakeWrist getOuttakeWristPosition(PropFindRightProcessor.pos propPos){
        if (propPos== PropFindRightProcessor.pos.LEFT){
            return CSCons.OuttakeWrist.flatRight;
        } else
            return CSCons.OuttakeWrist.flatLeft;

    }
}