//package org.firstinspires.ftc.masters;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
//import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//import java.util.Date;
//import java.util.List;
//
//@Config
//@Autonomous(name = "Center Stage Backdrop Red 2+2", group = "competition")
//public class CenterStageBackdropRed2PLUS2 extends LinearOpMode {
//    private OpenCvCamera webcam;
//
//
//    private static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
//    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution
//
//    TelemetryPacket packet = new TelemetryPacket();
//
//    enum State {
//        PURPLE_DEPOSIT_PATH,
//        PURPLE_DEPOSIT,
//
//        YELLOW_DEPOSIT_PATH,
//        YELLOW_DEPOSIT,
//
//        TOTRUSSLINE,
//        UNDERTRUSS,
//        TOSTACK,
//        STACKGRAB,
//
//        PARK,
//
//        END
//    }
//
//
//
//
//    int resetInt = 0;
//    int preloadInt = 0;
//    ElapsedTime depositTime = new ElapsedTime();
//    ElapsedTime waitTime = new ElapsedTime();
//    ElapsedTime preloadTime = new ElapsedTime();
//    ElapsedTime liftTime = new ElapsedTime();
//    SampleMecanumDrive drive;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "frontWebcam"), cameraMonitorViewId);
//        PropFindRight myPipeline;
//        webcam.setPipeline(myPipeline = new PropFindRight(telemetry,packet));
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//                /*
//                 * This will be called if the camera could not be opened
//                 */
//            }
//        });
//
//        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
//
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        }
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//
//
//        PropFindRight.pos propPos = null;
//
//        drive = new SampleMecanumDrive(hardwareMap);
//        Pose2d startPose = new Pose2d(new Vector2d(12, -58.5), Math.toRadians(90)); //Start position for roadrunner
//        drive.setPoseEstimate(startPose);
//
//        //PURPLE PIXEL
//
//        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(12, -58.5), Math.toRadians(90)))
//                .setTangent(Math.toRadians(40))
//                .splineToLinearHeading(new Pose2d(28, -24, Math.toRadians(180)), Math.toRadians(70))
//
//                .build();
//
//        TrajectorySequence middlePurple = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(12, -58.5), Math.toRadians(90)))
//                .setTangent(Math.toRadians(40))
//                .splineToLinearHeading(new Pose2d(26, -20, Math.toRadians(180)), Math.toRadians(70))
//
//                .build();
//
//        TrajectorySequence rightPurple = drive.trajectorySequenceBuilder(new Pose2d(new Vector2d(12, -58.5), Math.toRadians(90)))
//                .setTangent(Math.toRadians(40))
//                .splineToLinearHeading(new Pose2d(34.5, -27, Math.toRadians(180)), Math.toRadians(70))
//
//                .build();
//
//        //YELLOW PIXELS
//
//        TrajectorySequence leftyellow = drive.trajectorySequenceBuilder(leftPurple.end())
//                .setTangent(Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(46.5, -23, Math.toRadians(180)), Math.toRadians(0))
//
//                .build();
//
//
//        TrajectorySequence middleyellow = drive.trajectorySequenceBuilder(middlePurple.end())
//                .setTangent(Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(46.5, -31, Math.toRadians(180)), Math.toRadians(0))
//
//                .build();
//
//
//        TrajectorySequence rightyellow = drive.trajectorySequenceBuilder(rightPurple.end())
//                .setTangent(Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(46.5, -40, Math.toRadians(180)), Math.toRadians(0))
//
//                .build();
//
//        //OTHER PATHS
//
//        TrajectorySequence backAway = drive.trajectorySequenceBuilder(middleyellow.end())
//                .setTangent(Math.toRadians(-120))
//                .splineToLinearHeading(new Pose2d(10, -58, Math.toRadians(180)), Math.toRadians(-80))
//
//                .build();
//
//        TrajectorySequence tostacktruss = drive.trajectorySequenceBuilder(backAway.end())
//                .setTangent(Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(-24, -60, Math.toRadians(180)), Math.toRadians(180))
//
//                .build();
//
//        TrajectorySequence tostacklineup = drive.trajectorySequenceBuilder(tostacktruss.end())
//                .splineToLinearHeading(new Pose2d(-45, -40, Math.toRadians(170)), Math.toRadians(180))
//                .build();
//
//        TrajectorySequence Park = drive.trajectorySequenceBuilder(backAway.end())
//                .setTangent(Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(42, -58, Math.toRadians(180)), Math.toRadians(00))
//
//                .build();
//
//        State currentState;
//
//
//        int target=0;
//        int itarget=0;
//
//        drive.closeClaw();
//
//        propPos = myPipeline.position;
//
//        waitForStart();
//
//        long startTime = new Date().getTime();
//        long time = 0;
//
//        while (time < 50 && opModeIsActive()) {
//            time = new Date().getTime() - startTime;
//            propPos = myPipeline.position;
//            telemetry.addData("Position", propPos);
//        }
//
//        currentState = State.PURPLE_DEPOSIT_PATH;
//
//
//        while (opModeIsActive() && !isStopRequested()) {
//            drive.update();
//            drive.backSlidesMove(target);
////            drive.intakeSlidesMove(itarget);
//
//
//
//
//
//            switch (currentState) {
//                case PURPLE_DEPOSIT_PATH:
//
//                    if (propPos == PropFindRight.pos.LEFT) {
//                        drive.followTrajectorySequenceAsync(leftPurple);
//                        drive.intakeToGround();
//                            currentState = State.PURPLE_DEPOSIT;
//                    } else if (propPos == PropFindRight.pos.RIGHT) {
//                        drive.followTrajectorySequenceAsync(rightPurple);
//                        drive.intakeToGround();
//                        currentState = State.PURPLE_DEPOSIT;
//                    } else {
//                        drive.followTrajectorySequenceAsync(middlePurple);
//                        drive.intakeToGround();
//                        currentState = State.PURPLE_DEPOSIT;
//                    }
//
//                    break;
//
//                case PURPLE_DEPOSIT:
//                    if(propPos == PropFindRight.pos.LEFT) {
//                        if (preloadInt == 0) {
//                            if (!drive.isBusy()) {
//                                if (waitTime.milliseconds() > 1100) {
//                                    preloadTime.reset();
//
//                                    itarget = 1500;
//                                    preloadInt = 1;
//                                }
//
//                            }
//                        }
//                        if (preloadInt == 1) {
//
//
//                            if (preloadTime.milliseconds() > 1250) {
//                                drive.openClaw();
//                                drive.closeHook();
//                                currentState = State.YELLOW_DEPOSIT_PATH;
//                            }
//
//                        }
//                    }else if(propPos == PropFindRight.pos.RIGHT) {
//                        if (preloadInt == 0) {
//                            if (!drive.isBusy()) {
//                                if (waitTime.milliseconds() > 300) {
//                                    preloadTime.reset();
//                                    preloadInt = 1;
//                                }
//
//                            }
//                        }
//                        if (preloadInt == 1) {
//
//
//                            if (preloadTime.milliseconds() > 100) {
//                                drive.openClaw();
//                                drive.closeHook();
//                                currentState = State.YELLOW_DEPOSIT_PATH;
//                            }
//
//                        }
//                    } else{
//                        if (preloadInt == 0) {
//                            if (!drive.isBusy()) {
//                                if (waitTime.milliseconds() > 300) {
//                                    preloadTime.reset();
//                                    preloadInt = 1;
//                                }
//
//                            }
//                        }
//                        if (preloadInt == 1) {
//
//
//                            if (preloadTime.milliseconds() > 100) {
//                                drive.openClaw();
//                                drive.closeHook();
//                                currentState = State.YELLOW_DEPOSIT_PATH;
//                            }
//
//                        }
//
//                    }
//                    break;
//
//                case YELLOW_DEPOSIT_PATH:
//
//                    if(propPos == PropFindRight.pos.LEFT) {
//                        if(resetInt == 0) {
//                            preloadTime.reset();
//                            preloadInt = 0;
//
//
//
//                            resetInt++;
//                        }
//                        if (resetInt == 1) {
//                            if (preloadTime.milliseconds() > 1000) {
//                                drive.intakeToTransfer();
//                                drive.outtakeToBackdrop();
//                                itarget = 0;
//                                target = 900;
//
//                                drive.followTrajectorySequenceAsync(leftyellow);
//                                currentState = State.YELLOW_DEPOSIT;
//                            }
//                        }
//                    }else if(propPos == PropFindRight.pos.RIGHT) {
//                        if(resetInt == 0) {
//                            preloadTime.reset();
//                            preloadInt = 0;
//
//
//
//                            resetInt++;
//                        }
//                        if (resetInt == 1) {
//                            if (preloadTime.milliseconds() > 1000) {
//                                drive.intakeToTransfer();
//                                drive.outtakeToBackdrop();
//                                itarget = 0;
//                                target = 900;
//
//                                drive.followTrajectorySequenceAsync(rightyellow);
//                                currentState = State.YELLOW_DEPOSIT;
//                            }
//                        }
//                    }else{
//                        if(resetInt == 0) {
//                            preloadTime.reset();
//                            preloadInt = 0;
//
//
//
//                            resetInt++;
//                        }
//                        if (resetInt == 1) {
//                            if (preloadTime.milliseconds() > 1000) {
//                                drive.intakeToTransfer();
//                                drive.outtakeToBackdrop();
//                                itarget = 0;
//                                target = 900;
//
//                                drive.followTrajectorySequenceAsync(middleyellow);
//                                currentState = State.YELLOW_DEPOSIT;
//                            }
//                        }
//                    }
//
//
//                    break;
//
//                case YELLOW_DEPOSIT:
//                    if(!drive.isBusy()) {
//                        if (preloadInt == 0) {
//                            preloadTime.reset();
//                            drive.dropPixel();
//                            preloadInt++;
//                        }
//                        if(preloadInt == 1){
//                            if(preloadTime.milliseconds() > 1000) {
//                                target = 1500;
//                                currentState = State.TOTRUSSLINE;
//                            }
//
//                        }
//                    }
//                    break;
//
//                case TOTRUSSLINE:
//                    resetInt = 0;
//                    drive.followTrajectorySequenceAsync(backAway);
//                    currentState = State.UNDERTRUSS;
//                    break;
//
//                case UNDERTRUSS:
//                    if(!drive.isBusy()) {
//                        if(resetInt == 0) {
//                            drive.outtakeToTransfer();
//                            target = 0;
//                            liftTime.reset();
//                            resetInt++;
//                        }
//                        if(resetInt == 1) {
//
//                            if(liftTime.milliseconds() > 1000) {
//                                drive.followTrajectorySequenceAsync(tostacktruss);
//                                currentState = State.TOSTACK;
//                            }
//                        }
//                    }
//                    break;
//
//                case TOSTACK:
//
//                                drive.followTrajectorySequenceAsync(tostacklineup);
//                                currentState = State.END;
//
//                    break;
//
//                case PARK:
//                    if(!drive.isBusy()){
//
//                        drive.followTrajectorySequenceAsync(Park);
//
//
//                            currentState = State.END;
//
//
//                        }
//
//
//
//                    break;
//
//                case END:
//
//                    break;
//            }
//
//
//        }
//    }
//}