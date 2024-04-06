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
@Autonomous(name = "Backdrop blue 2 + 0", group = "competition")
public class BlueBackDrop_2_0 extends LinearOpMode {
    private OpenCvCamera webcam;


    private static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    TelemetryPacket packet = new TelemetryPacket();

    enum State {
        PURPLE_DEPOSIT_PATH,
        PURPLE_DEPOSIT,

        YELLOW_DEPOSIT_PATH,
        YELLOW_DEPOSIT,

        TOPARK,

        PARK,
        LOWER,
        END
    }




    int resetInt = 0;
    int preloadInt = 0;
    ElapsedTime purpleDepositTime = null;
    ElapsedTime depositTime = null;
    ElapsedTime waitTime = new ElapsedTime();
    ElapsedTime preloadTime = new ElapsedTime();
    ElapsedTime liftTime = new ElapsedTime();
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


        Pose2d startPose = new Pose2d(new Vector2d(16, 61.2), Math.toRadians(270)); //Start position for roadrunner
        drive.setPoseEstimate(startPose);
        drive.setWristServoPosition(outtakeWristPosition);

        //PURPLE PIXEL

        TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-40))
                .splineToLinearHeading(new Pose2d(12, 30, Math.toRadians(180)), Math.toRadians(-70))

                .build();

        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-40))
                .splineToLinearHeading(new Pose2d(35, 25, Math.toRadians(180)), Math.toRadians(-70))

                .build();

        TrajectorySequence middlePurple = drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(16, 34, Math.toRadians(-90)), Math.toRadians(-90))

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


        //YELLOW PIXELS

        TrajectorySequence leftyellow = drive.trajectorySequenceBuilder(leftPurple.end())
                .back(5)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(50, 42, Math.toRadians(180)), Math.toRadians(0))

                .build();


        TrajectorySequence middleyellow = drive.trajectorySequenceBuilder(middlePurple.end())
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(50, 34, Math.toRadians(180)), Math.toRadians(0))

                .build();


        TrajectorySequence rightyellow = drive.trajectorySequenceBuilder(rightPurple.end())
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(50, 29, Math.toRadians(180)), Math.toRadians(0))

                .build();

        //OTHER PATHS

        TrajectorySequence backAway = drive.trajectorySequenceBuilder(rightyellow.end())
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
        propPos= PropFindRightProcessor.pos.LEFT;

        currentState = State.PURPLE_DEPOSIT_PATH;


        while (opModeIsActive() && !isStopRequested()) {
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

                    currentState = State.PURPLE_DEPOSIT;

                    break;

                case PURPLE_DEPOSIT:
                    if (!drive.isBusy()){
                        if (purpleDepositTime ==null){
                            drive.raiseIntake();
                            outtakeWristPosition = CSCons.OuttakeWrist.flatRight;
                            purpleDepositTime = new ElapsedTime();
                        } else if (purpleDepositTime.milliseconds()>100) {
                            outtakeTarget = CSCons.OuttakePosition.AUTO.getTarget();
                            switch(propPos){
                                case RIGHT:
                                    drive.followTrajectorySequenceAsync(rightyellow);
                                    break;
                                case LEFT:
                                    drive.followTrajectorySequenceAsync(leftyellow);
                                    break;
                                case MID:
                                    drive.followTrajectorySequenceAsync(middleyellow);
                                    break;
                            }
                            currentState=State.YELLOW_DEPOSIT_PATH;
                        }
                    }
                    break;

                case YELLOW_DEPOSIT_PATH:

                    if (drive.getBackSlides().getCurrentPosition()>outtakeTarget- 200){
                        drive.outtakeToBackdrop();
                        drive.setWristServoPosition(CSCons.OuttakeWrist.flatRight);
                    } else if (drive.getBackSlides().getCurrentPosition()>10){
                        drive.outtakeToBackdrop();
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