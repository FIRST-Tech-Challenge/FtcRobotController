package org.firstinspires.ftc.masters.world;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.CSCons;
import org.firstinspires.ftc.masters.PropFindRightProcessor;
import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.Date;
import java.util.List;

@Config

public class BlueFarSide_2_0 extends LinearOpMode {
    private OpenCvCamera webcam;


    private static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    TelemetryPacket packet = new TelemetryPacket();

    enum State {
        PURPLE_DEPOSIT_PATH,
        PURPLE_DEPOSIT,

        TO_STACK,

        YELLOW_DEPOSIT_PATH,
        YELLOW_DEPOSIT,

        TOPARK,

        PARK,
        LOWER,
        END
    }




    int resetInt = 0;
    int preloadInt = 0;
    ElapsedTime purplePathTime = null;
    ElapsedTime purpleDropTime = null;
    ElapsedTime stackPickUpTime = null;
    ElapsedTime depositTime = null;
    ElapsedTime waitTime =null;
    ElapsedTime preloadTime = null;
    ElapsedTime liftTime = null;
    ElapsedTime pickupElapsedTime= null;
    CSCons.OuttakeWrist outtakeWristPosition = CSCons.OuttakeWrist.vertical;

    SampleMecanumDrive drive;

    Vector2d yellowLeftPos = new Vector2d();
    Vector2d yellowMidPos = new Vector2d();
    Vector2d yellowRightPos = new Vector2d();

    Pose2d tagAlignmentPosition = new Pose2d(54, 36, Math.toRadians(180));

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap, telemetry);
        drive.initializeAprilTagProcessing();
        drive.initializePropFindRightProcessing();
        drive.initializeVisionPortal(drive.getPropFindProcessor());


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        PropFindRightProcessor.pos propPos = null;


        Pose2d startPose = new Pose2d(new Vector2d(-39, 61.2), Math.toRadians(270)); //Start position for roadrunner
        drive.setPoseEstimate(startPose);
        drive.setWristServoPosition(outtakeWristPosition);

        //PURPLE PIXEL

        TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-50))
                .splineToLinearHeading(new Pose2d(-47, 15, Math.toRadians(-90)), Math.toRadians(-110))

                .build();

        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-110))
                .splineToLinearHeading(new Pose2d(-35, 35, Math.toRadians(-180)), Math.toRadians(-30))

                .build();

        TrajectorySequence middlePurple = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-50))
                .splineToLinearHeading(new Pose2d(-45, 14, Math.toRadians(-120)), Math.toRadians(-110))

                .build();

        TrajectorySequence rightPurpleToStack = drive.trajectorySequenceBuilder(rightPurple.end())
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-57, 11, Math.toRadians(181)), Math.toRadians(181))
                .build();

        TrajectorySequence leftPurpleToStack = drive.trajectorySequenceBuilder(leftPurple.end())
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-57, 11, Math.toRadians(181)), Math.toRadians(181))
                .build();

        TrajectorySequence midPurpleToStack = drive.trajectorySequenceBuilder(middlePurple.end())
                .setTangent(Math.toRadians(-140))
                .splineToLinearHeading(new Pose2d(-57, 11, Math.toRadians(181)), Math.toRadians(181))
                .build();


        TrajectorySequence stackToRightYellow = drive.trajectorySequenceBuilder(rightPurpleToStack.end())
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(10, 11.5, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(51, 33, Math.toRadians(180)), Math.toRadians(0))
                .build();

        TrajectorySequence stackToMidYellow = drive.trajectorySequenceBuilder(rightPurpleToStack.end())
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(20, 11.5, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(51, 34, Math.toRadians(180)), Math.toRadians(0))
                .build();

        TrajectorySequence stackToLeftYellow = drive.trajectorySequenceBuilder(rightPurpleToStack.end())
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(30, 11.5, Math.toRadians(180)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(51, 38, Math.toRadians(180)), Math.toRadians(0))
                .build();



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

        State currentState;


        int outtakeTarget=0;
        drive.raiseIntake();
        drive.closeFingers();


        propPos = drive.getPropFindProcessor().position;

        waitForStart();

        drive.dropIntake();

        long startTime = new Date().getTime();
        long time = 0;

        while (time < 50 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            propPos = drive.getPropFindProcessor().position;
            telemetry.addData("Position", propPos);
        }
        propPos= PropFindRightProcessor.pos.RIGHT;

        currentState = State.PURPLE_DEPOSIT_PATH;


        while (opModeIsActive() && !isStopRequested()) {
            telemetry.update();
            drive.update();
            drive.backSlidesMove(outtakeTarget);


            switch (currentState) {
                case PURPLE_DEPOSIT_PATH:

                    if (propPos == PropFindRightProcessor.pos.LEFT) {
                        drive.followTrajectorySequenceAsync(leftPurple);

                    } else if (propPos == PropFindRightProcessor.pos.RIGHT) {
                        drive.followTrajectorySequenceAsync(rightPurple);

                    } else {
                        drive.followTrajectorySequenceAsync(middlePurple);
                    }
                    purplePathTime = new ElapsedTime();
                    currentState = State.PURPLE_DEPOSIT;


                    break;

                case PURPLE_DEPOSIT:
                    if (purplePathTime!=null && purplePathTime.milliseconds()>200){
                        outtakeTarget= CSCons.OuttakePosition.LOW_AUTO.getTarget();
                        drive.setOuttakeToGround();
                    }
//                    if (Math.abs( drive.getBackSlides().getCurrentPosition())>90){
//                        telemetry.addData("outtake to ground", "true");
//                        drive.setOuttakeToGround();
//                    }
                    if (!drive.isBusy()){
                        drive.setOuttakeToGround();

                        if (purpleDropTime==null) {

                            purpleDropTime = new ElapsedTime();

                        } else if (purpleDropTime.milliseconds()>2000){
                            drive.openFrontFinger();
                        }
                        else if (purpleDropTime.milliseconds()>1250){
                            purplePathTime=null;
                            purpleDropTime=null;
                            outtakeTarget = 0;
                            drive.setOuttakeToTransfer();
                            drive.intakeOverStack();
                            drive.startIntake();
                            currentState= State.TO_STACK;
                            switch (propPos){
                                case LEFT:
                                    drive.followTrajectorySequenceAsync(leftPurpleToStack);
                                    break;
                                case RIGHT:
                                    drive.followTrajectorySequenceAsync(rightPurpleToStack);
                                    break;
                                case MID:
                                    drive.followTrajectorySequenceAsync(midPurpleToStack);
                                    break;
                            }

                        }


                    }

                    break;

                case TO_STACK:
                    if (drive.isBusy()) {
                        drive.setOuttakeToTransfer();
                    }
                    if (!drive.isBusy()){
                        if(pickupElapsedTime==null) {
                            drive.intakeToTopStack();
                            pickupElapsedTime = new ElapsedTime();
                        } else if (pickupElapsedTime.milliseconds()>1000){
                            drive.stopIntake();
                            drive.raiseIntake();
                            drive.outtakeToPickup();
                            pickupElapsedTime = new ElapsedTime();
                            currentState = State.YELLOW_DEPOSIT_PATH;
                            switch (propPos){
                                case LEFT:
                                    drive.followTrajectorySequenceAsync(stackToLeftYellow);
                                    break;
                                case RIGHT:
                                    drive.followTrajectorySequenceAsync(stackToRightYellow);
                                    break;
                                case MID:
                                    drive.followTrajectorySequenceAsync(stackToMidYellow);
                                    break;
                            }
                        }
                    }

                    break;

                case YELLOW_DEPOSIT_PATH:
                    if (pickupElapsedTime!=null &&  pickupElapsedTime.milliseconds()>250){
                        drive.closeFingers();
                        pickupElapsedTime =null;
                        drive.revertIntake();
                    }
                    if (drive.getPoseEstimate().getX()>30){
                        outtakeTarget = CSCons.OuttakePosition.AUTO.getTarget();
                        drive.closeFingers();
                        if (drive.getBackSlides().getCurrentPosition()>outtakeTarget- 200){
                            drive.outtakeToBackdrop();
                            drive.setWristServoPosition(CSCons.OuttakeWrist.flatLeft);
                        } else if (drive.getBackSlides().getCurrentPosition()>10){
                            drive.outtakeToBackdrop();
                        }
                        drive.stopIntake();
                    }

                    if (!drive.isBusy()){
                        drive.openFingers();
                        if (depositTime==null){
                            depositTime= new ElapsedTime();
                        } else if (depositTime.milliseconds()>100){
//                            drive.followTrajectorySequenceAsync(backAway);
//                            currentState= State.END;
                        }
                    }


                    break;

//                case YELLOW_DEPOSIT:
//                    if(!drive.isBusy()) {
//                        if (preloadInt == 0) {
//                            preloadTime.reset();
//                            drive.dropPixel();
//                            preloadInt++;
//                        }
//                        if(preloadInt == 1){
//                            if(preloadTime.milliseconds() > 1000) {
//                                outtakeTarget = 1700;
//                                currentState = State.TOPARK;
//                            }
//
//                        }
//                    }
//                    break;
//
//                case TOPARK:
//                    drive.followTrajectorySequenceAsync(backAway);
//                    currentState = State.PARK;
//                    break;
//
//                case PARK:
//                    if(!drive.isBusy()){
//
//                        drive.followTrajectorySequenceAsync(Park);
//
//                        drive.outtakeToTransfer();
//
//                        outtakeTarget = 0;
//                        currentState = State.END;
//
//
//                    }
//
//
//
//                    break;
//
                case END:

                    break;
            }


        }
    }
}