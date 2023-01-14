package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class AutonomousDrive extends LinearOpMode {
    // the camera
    private OpenCvWebcam webcam;
    // to store the red and blue values from the camera
    public static double red, blue;


    // we save the finishing angle for the field oriented after this op mode
    public static double lastAngle;
    // when turning off the op mode the imu turns off and his last value is 0, therefor we need to save the value before that (create a delay)
    public static double delayMaker;

    @Override
    public void runOpMode() {

    // initialize the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "cam"), cameraMonitorViewId);
        webcam.setPipeline(new PipeLine());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        // initialize the driveController (we do that after initializing the camera in order to enable "camera stream" in the drive controller)
        DriveController driveController = new DriveController(hardwareMap);

        Pose2d startPose = new Pose2d(-61.4, -37.4, 0);
        Pose2d scoringPose = new Pose2d(-4.5, -36.5, Math.toRadians(-76));

        Pose2d parking1 = new Pose2d(-13, -13.8, 0);
        Pose2d parking2 = new Pose2d(-13, -37.4, 0);
        Pose2d parking3 = new Pose2d(-13, -61  , 0);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        TrajectorySequence startToScore = drive.trajectorySequenceBuilder(startPose).lineToLinearHeading(scoringPose).build();

        TrajectorySequence scoringToParking1 = drive.trajectorySequenceBuilder(scoringPose).lineToLinearHeading(parking2).lineToLinearHeading(parking1).build();
        TrajectorySequence scoringToParking2 = drive.trajectorySequenceBuilder(scoringPose).lineToLinearHeading(parking2).build();
        TrajectorySequence scoringToParking3 = drive.trajectorySequenceBuilder(scoringPose).lineToLinearHeading(parking2).lineToLinearHeading(parking3).build();


        Thread elevatorController = new Thread(driveController::elevatorController);
        Thread Cycle = new Thread(driveController::cycle);

        waitForStart();
        resetRuntime();


        drive.followTrajectorySequence(startToScore);
        elevatorController.start();
        Cycle.start();

        while(Cycle.isAlive()){
            if (!opModeIsActive()) {
                Cycle.interrupt();
                elevatorController.interrupt();
                break;
            }
        }
        if (opModeIsActive()) {
            sleep(500);

            elevatorController.interrupt();

            if (red > 60) drive.followTrajectorySequence(scoringToParking3);
            else if (blue > 60) drive.followTrajectorySequence(scoringToParking2);
            else drive.followTrajectorySequence(scoringToParking1);

            telemetry.addData("time", getRuntime());
            telemetry.update();

            while (opModeIsActive()) {
                lastAngle = delayMaker;
                delayMaker = driveController.getRobotAngle();
            }
        }
    }

}

class PipeLine extends OpenCvPipeline {
    // the part of the image with the cone
    private Mat small;

    // the converted small image (rgb -> YCrCb)
    private final Mat YCrCbImage = new Mat();

    // the Cr and Cb channels of the YCrCbImage
    private final Mat RedChannel  = new Mat();
    private final Mat BlueChannel = new Mat();

    // the threshold bounded Cr and Cb channels
    private final Mat ThresholdRedImage  = new Mat();
    private final Mat ThresholdBlueImage = new Mat();

    // the part of the input image with the cone
    private final Rect coneWindow = new Rect(100, 60, 80, 100);

    // the color threshold
    private final Scalar thresholdMin = new Scalar(160, 160, 160);
    private final Scalar thresholdMax = new Scalar(255, 255, 255);

    @Override
    public Mat processFrame(Mat input){
        // get the part of the input image with the cone
        small = input.submat(coneWindow);

        // convert the small image (rgb -> YCrCb)
        Imgproc.cvtColor(small, YCrCbImage, Imgproc.COLOR_RGB2YCrCb);

        // get the Cr and Cb channels of the YCrCbImage
        Core.extractChannel(YCrCbImage, RedChannel , 1);
        Core.extractChannel(YCrCbImage, BlueChannel, 2);

        // threshold the Cr and Cb channels
        Core.inRange(RedChannel , thresholdMin, thresholdMax, ThresholdRedImage );
        Core.inRange(BlueChannel, thresholdMin, thresholdMax, ThresholdBlueImage);

        // count the red and blue pixels and place them into the red and blue fields from AutonomousDrive
        AutonomousDrive.red  = Core.mean(ThresholdRedImage ).val[0];
        AutonomousDrive.blue = Core.mean(ThresholdBlueImage).val[0];

        // visual que
        if       (AutonomousDrive.red > 60) Imgproc.rectangle(small, new Point(0, 0), new Point(79.0, 99.0), new Scalar(255, 0  , 0  ));
        else if (AutonomousDrive.blue > 60) Imgproc.rectangle(small, new Point(0, 0), new Point(79.0, 99.0), new Scalar(0  , 0  , 255));
        else                                Imgproc.rectangle(small, new Point(0, 0), new Point(79.0, 99.0), new Scalar(255, 255, 255));


        // show the small image
        return small;
    }
}