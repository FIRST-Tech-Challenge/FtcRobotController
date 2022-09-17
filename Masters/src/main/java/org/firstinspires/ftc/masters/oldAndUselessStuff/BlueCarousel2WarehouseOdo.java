package org.firstinspires.ftc.masters.oldAndUselessStuff;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.ContourDetectionPipeline;
import org.firstinspires.ftc.masters.FreightFrenzyConstants;
import org.firstinspires.ftc.masters.MultipleCameraCV;
import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
import org.firstinspires.ftc.masters.trajectorySequence.TrajectorySequence;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Date;

@Autonomous(name = "Blue - Carousel 2 Warehouse (STATE)", group = "competition")
public class BlueCarousel2WarehouseOdo extends LinearOpMode {

    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    public static double borderLeftX    = 0.0;   //fraction of pixels from the left side of the cam to skip
    public static double borderRightX   = 0.0;   //fraction of pixels from the right of the cam to skip
    public static double borderTopY     = 0.0;   //fraction of pixels from the top of the cam to skip
    public static double borderBottomY  = 0.0;   //fraction of pixels from the bottom of the cam to skip

    // Pink Range                                      Y      Cr     Cb
    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 160.0, 100.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);
    final int SERVO_DROP_PAUSE=900;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, this, telemetry);

        drive.openCVInnitShenanigans();
        MultipleCameraCV.ShippingElementDeterminationPipeline.ElementPosition freightLocation = null;
        freightLocation = drive.analyze();

        Pose2d startPose = new Pose2d(new Vector2d(-35, 63), Math.toRadians(270));

        drive.setPoseEstimate(startPose);

        drive.linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_LIFT);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        //OpenCV Pipeline
        ContourDetectionPipeline myPipeline;
        webcam.setPipeline(myPipeline = new ContourDetectionPipeline(borderLeftX,borderRightX,borderTopY,borderBottomY));
        // Configuration of Pipeline
        myPipeline.configureScalarLower(scalarLowerYCrCb.val[0],scalarLowerYCrCb.val[1],scalarLowerYCrCb.val[2]);
        myPipeline.configureScalarUpper(scalarUpperYCrCb.val[0],scalarUpperYCrCb.val[1],scalarUpperYCrCb.val[2]);
        // Webcam Streaming
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

        // Only if you are using ftcdashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(webcam, 10);

        waitForStart();

        ElapsedTime elapsedTime = new ElapsedTime();

        long startTime = new Date().getTime();
        long time = 0;

        while (time < 200 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            freightLocation = drive.analyze();

            telemetry.addData("Position", freightLocation);
            telemetry.update();
        }
        switch (freightLocation) {
            case LEFT:
                drive.linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_LOW);
                break;
            case MIDDLE:
                drive.linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_MIDDLE);
                break;
            case RIGHT:
                drive.linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_TOP);
                break;
            default:
                drive.linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_TOP);
        }
        drive.linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.linearSlideMotor.setPower(.8);

        if (isStopRequested()) return;


//      Deposit initial freight
        Pose2d hubPosition = new Pose2d(new Vector2d(-23, 38), Math.toRadians(360-47));

        TrajectorySequence toHubHigh = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(hubPosition)
                .build();
        TrajectorySequence toHubLow = drive.trajectorySequenceBuilder(startPose)
                .lineToSplineHeading(new Pose2d(new Vector2d(-22, 36.5), Math.toRadians(360-47)))
                .build();

        switch (freightLocation) {
            case LEFT:
                drive.followTrajectorySequence(toHubLow);
                break;
            case MIDDLE:
                drive.followTrajectorySequence(toHubHigh);
                break;
            case RIGHT:
                drive.followTrajectorySequence(toHubHigh);
                break;
        }

        drive.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_DROP);
        drive.pause(SERVO_DROP_PAUSE);
        drive.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_BOTTOM);
        if (freightLocation== MultipleCameraCV.ShippingElementDeterminationPipeline.ElementPosition.LEFT){
            drive.pause(300);
        }
        drive.stopShippingElementCamera();
        drive.retract();

//        To spin duck

        Pose2d position = drive.getLocalizer().getPoseEstimate();

        TrajectorySequence toCarousel = drive.trajectorySequenceBuilder(position)
                .lineToLinearHeading(new Pose2d( new Vector2d(-58.5, 58.5), Math.toRadians(0)))
                .build();
        drive.followTrajectorySequence(toCarousel);

        drive.intakeMotor.setPower(1);
        drive.jevilTurnBlueCarousel(3);

        TrajectorySequence leaveCarousel = drive.trajectorySequenceBuilder(drive.getLocalizer().getPoseEstimate())
                .lineTo(new Vector2d(-55, 47))
                .turn(Math.toRadians(-90))
                .build();
        drive.followTrajectorySequence(leaveCarousel);

        // drive.turn(Math.toRadians(-90));

//        TrajectorySequence AcquireDuck = drive.trajectorySequenceBuilder(drive.getLocalizer().getPoseEstimate())
//                .lineTo(new Vector2d(-55, 63))
//                .build();
//        drive.followTrajectorySequence(AcquireDuck);

        drive.findDuckBlue();


        position = drive.getLocalizer().getPoseEstimate();

        drive.pause(350);
        drive.intakeMotor.setPower(-0.8);
        drive.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_LIFT);
        drive.pause(250);
        drive.linearSlideMotor.setTargetPosition(FreightFrenzyConstants.SLIDE_TOP);
        drive.linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.linearSlideMotor.setPower(.7);
        drive.intakeMotor.setPower(0);

        position = position.minus(new Pose2d(12,0,0));
        TrajectorySequence depositDuck = drive.trajectorySequenceBuilder(position)
                .lineTo(new Vector2d(-50, 55))
                .splineToLinearHeading(hubPosition, Math.toRadians(360-47))
                .build();
        drive.followTrajectorySequence(depositDuck);

        drive.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_DROP);
        drive.pause(SERVO_DROP_PAUSE);
        drive.linearSlideServo.setPosition(FreightFrenzyConstants.DUMP_SERVO_BOTTOM);

        drive.retract();

//        //This is the stuff in the last one.
//        position = drive.getLocalizer().getPoseEstimate();
//        TrajectorySequence trajSeq7 = drive.trajectorySequenceBuilder(position)
////                .strafeRight(20)
//                .lineTo(new Vector2d(-32, 43))
//                .splineToLinearHeading (new Pose2d(new Vector2d(-62, 35),Math.toRadians(270)), Math.toRadians(180))
//                .build();
//        drive.followTrajectorySequence(trajSeq7);
        TrajectorySequence fromHubToWaitPos = drive.trajectorySequenceBuilder(drive.getLocalizer().getPoseEstimate())
                .lineToSplineHeading(new Pose2d(new Vector2d(-24, 60), Math.toRadians(180)))
                .build();
        TrajectorySequence fromWaitPosToWarehouse = drive.trajectorySequenceBuilder(fromHubToWaitPos.end())
                .lineToLinearHeading(new Pose2d( new Vector2d(48, 67), Math.toRadians(180)))
                .build();


        drive.followTrajectorySequence(fromHubToWaitPos);
        double seconds = elapsedTime.seconds();
        while (seconds<26) {
            seconds = elapsedTime.seconds();
        }
        drive.followTrajectorySequence(fromWaitPosToWarehouse);
      //  drive.getCube(2000);

    }


}


// MeepMeepTesting Code &, more importantly, Vectors and Poses

///drive.trajectorySequenceBuilder(new Pose2d(-35, 63, Math.toRadians(270)))
//        .lineToSplineHeading(new Pose2d(new Vector2d(-12.5, 42), Math.toRadians(270)))
//        .lineToLinearHeading(new Pose2d( new Vector2d(-60, 60), Math.toRadians(0)))
//        .lineTo(new Vector2d(-55, 55))
//        .lineToLinearHeading(new Pose2d(-55, 54, Math.toRadians(270)))
//        .lineTo(new Vector2d(-55, 63))
//        .lineTo(new Vector2d(-50, 55))
//        .splineToLinearHeading(new Pose2d(-12.5, 42, Math.toRadians(270)), Math.toRadians(270))
//        .strafeRight(20)
//        //.lineTo(new Vector2d(-22, 44))
//        .splineToLinearHeading (new Pose2d(new Vector2d(-62, 35),Math.toRadians(270)), Math.toRadians(180))
//        .build() );