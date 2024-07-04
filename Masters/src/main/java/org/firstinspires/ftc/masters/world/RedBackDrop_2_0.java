package org.firstinspires.ftc.masters.world;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.masters.CSCons;
import org.firstinspires.ftc.masters.PropFindRightProcessor;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;
import org.firstinspires.ftc.masters.world.paths.BlueBackDropPath;
import org.firstinspires.ftc.masters.world.paths.RedBackDropPath;

@Config
@Autonomous(name = "Red Backdrop 2 + 0", group = "competition")
public class RedBackDrop_2_0 extends BackDropOpMode {

    Pose2d tagAlignmentPosition = new Pose2d(54, 36, Math.toRadians(180));

    @Override
    protected void initializeProp(){
        drive.initializePropFindRightProcessing();
    }
    @Override
    public void runOpMode() throws InterruptedException {

        initAuto();

        Pose2d startPose = new Pose2d(new Vector2d(16, -61.2), Math.toRadians(90)); //Start position for roadrunner
        drive.setPoseEstimate(startPose);
        drive.setWristServoPosition(outtakeWristPosition);

        //PURPLE PIXEL

        rightPurple = RedBackDropPath.getRightPurple(drive, startPose);
        leftPurple = RedBackDropPath.getLeftPurple(drive, startPose);
        middlePurple = RedBackDropPath.getMidPurple(drive, startPose);

//
//        TrajectorySequence tagAlignLeft = drive.trajectorySequenceBuilder(leftPurple.end())
//                .back(5)
//                .setTangent(Math.toRadians(0))
//                .splineToLinearHeading(tagAlignmentPosition, Math.toRadians(0))
//                .build();
//
//        TrajectorySequence tagAlignMid = drive.trajectorySequenceBuilder(middlePurple.end())
//                .setTangent(Math.toRadians(0))
//                .splineToLinearHeading(tagAlignmentPosition, Math.toRadians(0))
//                .build();
//
//        TrajectorySequence tagAlignRight = drive.trajectorySequenceBuilder(rightPurple.end())
//                .back(5)
//                .setTangent(Math.toRadians(0))
//                .splineToLinearHeading(tagAlignmentPosition, Math.toRadians(0))
//                .build();


        //YELLOW PIXELS

        leftYellow = RedBackDropPath.getLeftYellow(drive, leftPurple.end());
        midYellow = RedBackDropPath.getMidYellow(drive, middlePurple.end());
        rightYellow = RedBackDropPath.getRightYellow(drive, rightPurple.end());

        //OTHER PATHS

        parkFromLeft = RedBackDropPath.park(drive, leftYellow.end());
        parkFromRight =RedBackDropPath.park(drive, rightYellow.end());
        parkFromMid = RedBackDropPath.park(drive, midYellow.end());


        drive.raiseIntake();
        drive.closeFingers();

        propPos = drive.getPropFindProcessor().position;

        waitForStart();

        currentState = BackDropOpMode.State.PURPLE_DEPOSIT_PATH;
        drive.dropIntake();

        retrievePropPos();


//TO DO: go park

        while (opModeIsActive() && !isStopRequested()) {
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
        return RedBackDropPath.toStackWing(drive, robotPosition);
    }

    @Override
    public CSCons.OuttakeWrist getOuttakeWristPosition(){
        return CSCons.OuttakeWrist.flatLeft;
    }
}