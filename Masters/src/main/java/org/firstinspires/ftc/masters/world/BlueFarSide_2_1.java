package org.firstinspires.ftc.masters.world;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.PropFindRightProcessor;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;
import org.firstinspires.ftc.masters.world.paths.BlueFarSidePath;

@Config
@Autonomous(name = "Far Side blue 2 + 1", group = "competition")
public class BlueFarSide_2_1 extends FarSideOpMode {


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

        parkFromMid = BlueFarSidePath.park(drive, stackToMidYellow.end());
        parkFromLeft = BlueFarSidePath.park(drive, stackToLeftYellow.end());
        parkFromRight = BlueFarSidePath.park(drive, stackToRightYellow.end());

        propPos = drive.getPropFindProcessor().position;

        waitForStart();

        currentState = State.PURPLE_DEPOSIT_PATH;

        retrievePropPos();

        TrajectorySequence nextPath=null;

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            drive.backSlidesMove(outtakeTarget);
            if (retractElapsed != null) {
                telemetry.addData("time", retractElapsed.milliseconds());
            }
            telemetry.update();

            switch (currentState){
                case PURPLE_DEPOSIT_PATH:
                    purpleDepositPath();
                    break;
                case PURPLE_DEPOSIT:
                    purpleDeposit();
                    break;
                case TO_STACK:

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

                    toStack(nextPath);
                    break;
                case BACKDROP_DEPOSIT_PATH:
                    switch (propPos){
                        case LEFT:
                            backdropDepositPath(State.PARK, parkFromLeft);
                            break;
                        case RIGHT:
                            backdropDepositPath(State.PARK, parkFromRight);
                            break;
                        case MID:
                            backdropDepositPath(State.PARK, parkFromMid);
                            break;
                    }

                    break;
                case PARK:

                    park();
                    break;

            }


        }
    }
}