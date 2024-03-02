package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Date;
import java.util.List;

@Config
@Autonomous(name = "Center Stage Stack Side Blue", group = "competition")
public class CenterStagePilesBlue extends LinearOpMode {
    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    enum State {
        PURPLE_DEPOSIT_PATH,
//        PURPLE_DEPOSIT_PATH1,
//        PURPLE_DEPOSIT_PATH2,
        PURPLE_DEPOSIT,
        RETRACT_SLIDE,
        DRIVE_TO_STACK,
        PICK_UP_FROM_STACK,

        UNTURN,
        BACKUP_FROM_SPIKES,

        YELLOW_DEPOSIT_PATH1,
        YELLOW_DEPOSIT_PATH2,
        YELLOW_DEPOSIT_PATH,
        YELLOW_DEPOSIT_STRAIGHT,
        YELLOW_DEPOSIT,
        BACK,
        PARK,
        STOP
    }

    ElapsedTime depositTime = new ElapsedTime();
    int resetInt = 0;

    SampleMecanumDrive drive;

    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "frontWebcam"), cameraMonitorViewId);
        PropFindLeft myPipeline;
        webcam.setPipeline(myPipeline = new PropFindLeft(telemetry,packet));
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //CenterStageComputerVisionPipelines CV = new CenterStageComputerVisionPipelines(hardwareMap, telemetry);
        PropFindLeft.pos propPos = null;

        drive = new SampleMecanumDrive(hardwareMap, telemetry);
        Pose2d startPose = new Pose2d(new Vector2d(-35, 58.5), Math.toRadians(270)); //Start position for roadrunner
        drive.setPoseEstimate(startPose);

        State currentState;


        TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-40, 50, Math.toRadians(110+180)))
                .build();

        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-40, 50, Math.toRadians(110+180)))
                .build();

        TrajectorySequence centerPurple = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(-40, 50, Math.toRadians(110+180)))
                .build();

        TrajectorySequence rightPurpleToStack = drive.trajectorySequenceBuilder(rightPurple.end())
                .lineToSplineHeading(new Pose2d(-40, 10, Math.toRadians(180)))
                .build();

        TrajectorySequence leftPurpleToStack = drive.trajectorySequenceBuilder(leftPurple.end())
                .lineToSplineHeading(new Pose2d(-40, 10, Math.toRadians(180)))
                .build();

        TrajectorySequence straightToBackBoard = drive.trajectorySequenceBuilder(rightPurpleToStack.end())
                .lineToLinearHeading(new Pose2d(50, 10, Math.toRadians(180)))
                .build();

        TrajectorySequence strafeToBoardRight = drive.trajectorySequenceBuilder(straightToBackBoard.end())
                .strafeTo(new Vector2d(50, 24))
                .build();

        TrajectorySequence strafeToBoardCenter = drive.trajectorySequenceBuilder(straightToBackBoard.end())
                .strafeTo(new Vector2d(50, 30))
                .build();

        TrajectorySequence strafeToBoardLeft = drive.trajectorySequenceBuilder(straightToBackBoard.end())
                .strafeTo(new Vector2d(50, 36))
                .build();


        TrajectorySequence toStackFromRight = drive.trajectorySequenceBuilder(strafeToBoardRight.end())
                .splineToConstantHeading(new Vector2d(40, 10),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-40, 10),Math.toRadians(180))
                .build();

        TrajectorySequence toStackFromCenter = drive.trajectorySequenceBuilder(strafeToBoardCenter.end())
                .splineToConstantHeading(new Vector2d(40, 10),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-40, 10),Math.toRadians(180))
                .build();

        TrajectorySequence toStackFromLeft = drive.trajectorySequenceBuilder(strafeToBoardLeft.end())
                .splineToConstantHeading(new Vector2d(40, 10),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-40, 10),Math.toRadians(180))
                .build();




        Trajectory purpleDepositPathC = drive.trajectoryBuilder(startPose,false)
                .lineToSplineHeading(new Pose2d(-35, 10, Math.toRadians(90)))
                .build();

        Trajectory purpleDepositPathL = drive.trajectoryBuilder(purpleDepositPathC.end(),false)
                .lineToSplineHeading(new Pose2d(-32, 17, Math.toRadians(43)))
                .build();

        Trajectory purpleDepositPathR = drive.trajectoryBuilder(purpleDepositPathC.end(),false)
                .lineToSplineHeading(new Pose2d(-38, 17, Math.toRadians(120)))
                .build();



        Trajectory backUpFromSpikes = drive.trajectoryBuilder(purpleDepositPathC.end(),false)
                .forward(.5)
                .build();

        Trajectory yellowDepositPath1 = drive.trajectoryBuilder(backUpFromSpikes.end(),false)
                .lineToSplineHeading(new Pose2d(-35, 5, Math.toRadians(180)))
                .build();

        Trajectory yellowDepositPath2 = drive.trajectoryBuilder(yellowDepositPath1.end(),false)
                .back(65)
                .build();

        Trajectory yellowDepositPathC = drive.trajectoryBuilder(yellowDepositPath2.end(),false)
                .splineToLinearHeading(new Pose2d(46, 34, Math.toRadians(180)), Math.toRadians(0))
                .build();

        Trajectory yellowDepositPathL = drive.trajectoryBuilder(yellowDepositPath2.end(),false)
                .splineToLinearHeading(new Pose2d(46, 37, Math.toRadians(180)), Math.toRadians(0))
                .build();

        Trajectory yellowDepositPathR = drive.trajectoryBuilder(yellowDepositPath2.end(),false)
                .splineToLinearHeading(new Pose2d(48, 30, Math.toRadians(180)), Math.toRadians(0))
                .build();

        Trajectory park;

        int backSlidesTarget=0;
        int intakeTarget =0;
        boolean CLAWEXTENDAHHHH = false;

        drive.closeClaw();

        propPos = myPipeline.position;

        telemetry.addData("Position", propPos);
        telemetry.update();

        waitForStart();

        drive.closeClaw();
        int previousIntakeTarget = 0;
        ElapsedTime elapsedTime= new ElapsedTime();


        long startTime = new Date().getTime();
        long time = 0;

        while (time < 50 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            propPos = myPipeline.position;
            telemetry.addData("Position", propPos);
        }

        currentState = State.PURPLE_DEPOSIT_PATH;

        if (propPos == PropFindLeft.pos.LEFT) {
            drive.followTrajectorySequenceAsync(leftPurple);
            drive.intakeToGround();
        } else if (propPos == PropFindLeft.pos.RIGHT) {
            drive.followTrajectorySequenceAsync(rightPurple);
            drive.intakeToGround();
        } else {
            drive.followTrajectorySequenceAsync(centerPurple);
            drive.intakeToGround();
        }

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            drive.backSlidesMove(backSlidesTarget);
            drive.intakeSlidesMove(intakeTarget);

            telemetry.addData("intake slides", drive.getIntakeSlides().getCurrentPosition());
            telemetry.addData("current state", currentState.name());
            telemetry.addData("Claw", CLAWEXTENDAHHHH);

            switch (currentState) {

                case PURPLE_DEPOSIT_PATH:

                    if (!drive.isBusy()){
                        intakeTarget = 1100;
                        currentState = State.PURPLE_DEPOSIT;
                    }
                break;


//                case PURPLE_DEPOSIT_PATH1:
//                    if (!drive.isBusy()) {
//                        currentState = State.PURPLE_DEPOSIT_PATH2;
//                        if (propPos == PropFindLeft.pos.LEFT) {
//                            drive.followTrajectoryAsync(purpleDepositPathL);
//                        } else if (propPos == PropFindLeft.pos.RIGHT) {
//                            drive.followTrajectoryAsync(purpleDepositPathR);
//                        }
//
//                    }
//                    break;
//                case PURPLE_DEPOSIT_PATH2:
//                    if (!drive.isBusy()) {
//                        currentState = State.PURPLE_DEPOSIT;
//
//                    } else {
//                        //drive.intakeToGround();
//                    }
//                    break;

                case PURPLE_DEPOSIT:

                if (!CLAWEXTENDAHHHH && drive.getIntakeSlides().getCurrentPosition()>intakeTarget-20 && drive.getIntakeSlides().getCurrentPosition()<intakeTarget+20){
                        drive.openClaw();
                        elapsedTime = new ElapsedTime();
                        CLAWEXTENDAHHHH = true;
                    }
                    if (CLAWEXTENDAHHHH && elapsedTime.milliseconds()>200){
                        telemetry.addData("WHYYYYY", "INSERT SWEAR WORDS IN FUTURE UPDATE");
                        previousIntakeTarget = intakeTarget;
                        intakeTarget = 0;
                    }
                    if (CLAWEXTENDAHHHH && elapsedTime.milliseconds() > 300 && drive.getIntakeSlides().getCurrentPosition()<previousIntakeTarget-100){
                        drive.intakeToTransfer();
                        currentState= State.RETRACT_SLIDE;
                    }

                    break;
                case RETRACT_SLIDE:
                    if (drive.getIntakeSlides().getCurrentPosition()<50){
                        if (propPos == PropFindLeft.pos.LEFT){
                            drive.followTrajectorySequenceAsync(leftPurpleToStack);
                        } else if (propPos== PropFindLeft.pos.RIGHT){
                            drive.followTrajectorySequenceAsync(leftPurpleToStack);
                        } else {
                            drive.followTrajectorySequenceAsync(leftPurpleToStack);
                        }
                        currentState = State.DRIVE_TO_STACK;
                    }

                    break;

                case DRIVE_TO_STACK:
                    if (!drive.isBusy()){
                        drive.intakeToTopStack();
                        intakeTarget = 1400;
                    }




                    break;




                case BACKUP_FROM_SPIKES:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(yellowDepositPath1);
                        currentState = State.YELLOW_DEPOSIT_PATH1;
                    }
                    break;
                case YELLOW_DEPOSIT_PATH1:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(yellowDepositPath2);
                        currentState = State.YELLOW_DEPOSIT_PATH2;
                    }
                    break;
                case YELLOW_DEPOSIT_PATH2:
                    if (!drive.isBusy()) {
                        if (propPos == PropFindLeft.pos.LEFT){
                            drive.followTrajectoryAsync(yellowDepositPathL);
                        } else if (propPos == PropFindLeft.pos.RIGHT){
                            drive.followTrajectoryAsync(yellowDepositPathR);
                        } else if (propPos == PropFindLeft.pos.MID){
                            drive.followTrajectoryAsync(yellowDepositPathC);
                        }

                        currentState = State.YELLOW_DEPOSIT_PATH;
                        //currentState = State.STOP;
                    } else {
//                        target= CSCons.OuttakePosition.LOW.getTarget();
//                        drive.intakeToTransfer();
//                        drive.outtakeToBackdrop();
                    }
                    break;
                case YELLOW_DEPOSIT_PATH:
                    if (!drive.isBusy() ) {
                        currentState = State.YELLOW_DEPOSIT_STRAIGHT;
                    } else {
//                        target= CSCons.OuttakePosition.LOW.getTarget();
//                        drive.intakeToTransfer();
//                        drive.outtakeToBackdrop();
                    }
                    break;
                case YELLOW_DEPOSIT_STRAIGHT:
                    if(!drive.isBusy()){
                        Trajectory straight= drive.trajectoryBuilder(drive.getPoseEstimate())
                                .back(3).build();
                        drive.followTrajectoryAsync(straight);

                        currentState= State.YELLOW_DEPOSIT;
                    }
                    break;
                case YELLOW_DEPOSIT:

                    if(resetInt == 0){
                        depositTime.reset();
                        resetInt++;
                    }
                    if(resetInt == 1){
                    if (!drive.isBusy()) {

                        park = drive.trajectoryBuilder(drive.getPoseEstimate(), false)
                                .back(4)
                                .build();
                        //april tag alignment
                        //if april tag is aligned drop and
                        if (depositTime.milliseconds() > 500) {
                            drive.dropPixel();
                        }
                        if (depositTime.milliseconds() > 700) {
                            drive.followTrajectoryAsync(park);
                            currentState = State.BACK;

                        }
                    }
                    }
                    break;
                case BACK:
                    if (!drive.isBusy()) {
                        //drive.outtakeToTransfer();
                        backSlidesTarget = 0;
                        park = drive.trajectoryBuilder(drive.getPoseEstimate(),false)
                                .splineToLinearHeading(new Pose2d(new Vector2d(50, 56), Math.toRadians(180)), Math.toRadians(0))
                                .build();
                        //drive.followTrajectoryAsync(park);
                        currentState= State.PARK;
                    }
                    break;
                case PARK:
                    if (!drive.isBusy()) {
                       // drive.outtakeToTransfer();
                       backSlidesTarget=0;
                    }
                    break;
                case STOP:
                    break;
            }
        }
    }
}