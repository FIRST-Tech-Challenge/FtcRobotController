package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


/*

To Do:

1) drop purple pixel method
2) incorporate AprilTags
3) openCV split screen stuff to detect team prop location
4) if splines do not work, switch to forward(), strafeRight(), and strafeLeft()

 */



@Config
@Autonomous(name = "MainAutonomous")
public class MainAuto extends LinearOpMode{

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    Bot bot;

    enum Side {
        RED, BLUE, NULL;
    }
    enum DistanceToBackdrop{
        CLOSE, FAR, NULL;
    }

    enum AutoPath{
        MECHANICAL_FAILURE, NO_SENSE, OPTIMAL;
    }

    Side side = Side.NULL;
    DistanceToBackdrop dtb= DistanceToBackdrop.NULL;
    AutoPath autopath = AutoPath.OPTIMAL;

    double fx = 1078.03779;
    double fy = 1084.50988;
    double cx = 580.850545;
    double cy = 245.959325;

    // UNITS ARE METERS
    double tagsize = 0.032; //ONLY FOR TESTING



    @Override
    public void runOpMode() throws InterruptedException {
        bot = Bot.getInstance(this);
        GamepadEx gp1 = new GamepadEx(gamepad1);

        Pose2d startPoseBlueFar = new Pose2d(-54, -36, 0);
        Pose2d startPoseBlueClose = new Pose2d(-54, 36, 0);
        Pose2d startPoseRedClose = new Pose2d(54, 36, 0);
        Pose2d startPoseRedFar = new Pose2d(54, -36, 0);


        //CAMERA STUFF =====================

        WebcamName camName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(camName);
        AprilTagsPipeline aprilTagPipeline = new AprilTagsPipeline(tagsize, fx, fy, cx, cy);


        camera.setPipeline(aprilTagPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });



        while (!isStarted()) {
            gp1.readButtons();
            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                side = Side.RED;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                side = Side.BLUE;
            }
            if(gp1.wasJustPressed(GamepadKeys.Button.X)){
                dtb= DistanceToBackdrop.CLOSE;
            }
            if(gp1.wasJustPressed(GamepadKeys.Button.Y)){
                dtb= DistanceToBackdrop.FAR;
            }
            if(gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
                autopath= AutoPath.MECHANICAL_FAILURE;
            }
            if(gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
                autopath= AutoPath.NO_SENSE;
            }

            if(dtb== DistanceToBackdrop.FAR && side==Side.BLUE){
                drive.setPoseEstimate(startPoseBlueFar);
            }

            if(dtb== DistanceToBackdrop.FAR && side==Side.RED){
                drive.setPoseEstimate(startPoseRedFar);
            }
            if(dtb== DistanceToBackdrop.CLOSE && side==Side.BLUE){
                drive.setPoseEstimate(startPoseBlueClose);
            }
            if(dtb== DistanceToBackdrop.CLOSE && side==Side.RED){
                drive.setPoseEstimate(startPoseRedClose);
            }


            TrajectorySequence blueAllianceFar = drive.trajectorySequenceBuilder(startPoseBlueFar)
                    .splineTo(new Vector2d(-36,-36), Math.toRadians(0))
                    .addTemporalMarker(this::dropPurplePixel)
                    .splineTo(new Vector2d(-36,48), Math.toRadians(90))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, this::outtake)
                    .splineTo(new Vector2d(-72,48), Math.toRadians(90))
                    .build();

            TrajectorySequence redAllianceFar= drive.trajectorySequenceBuilder(startPoseRedFar)
                    .splineTo(new Vector2d(36,-36), Math.toRadians(0))
                    .addTemporalMarker(this::dropPurplePixel)
                    .splineTo(new Vector2d(36,48), Math.toRadians(-90))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, this::outtake)
                    .splineTo(new Vector2d(72,48), Math.toRadians(-90))
                    .build();

            TrajectorySequence blueAllianceClose = drive.trajectorySequenceBuilder(startPoseBlueClose)
                    .splineTo(new Vector2d(-54,20), Math.toRadians(0))
                    .splineTo(new Vector2d(-36,20), Math.toRadians(90))
                    .addTemporalMarker(this::dropPurplePixel)
                    .splineTo(new Vector2d(-36,48), Math.toRadians(90))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, this::outtake)
                    .splineTo(new Vector2d(-72,48), Math.toRadians(90))
                    .build();

            TrajectorySequence redAllianceClose= drive.trajectorySequenceBuilder(startPoseRedClose)
                    .splineTo(new Vector2d(54,12), Math.toRadians(0))
                    .splineTo(new Vector2d(36,12), Math.toRadians(-90))
                    .addTemporalMarker(this::dropPurplePixel)
                    .splineTo(new Vector2d(36,48), Math.toRadians(-90))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, this::outtake)
                    .splineTo(new Vector2d(72,48), Math.toRadians(90))
                    .build();


            //thread will enable the robot to move forward while performing other activities
            Thread blueAllianceFarThread = new Thread(() -> drive.followTrajectorySequence(blueAllianceFar));
            Thread redAllianceFarThread = new Thread(() -> drive.followTrajectorySequence(redAllianceFar));
            Thread blueAllianceCloseThread = new Thread(() -> drive.followTrajectorySequence(blueAllianceClose));
            Thread redAllianceCloseThread = new Thread(() -> drive.followTrajectorySequence(redAllianceClose));

            waitForStart();
            if (!isStopRequested()) {


                if(dtb== DistanceToBackdrop.FAR && side==Side.BLUE && autopath==AutoPath.NO_SENSE){
                    blueAllianceFarThread.start();
                    sleep(1000);
                    blueAllianceFarThread.interrupt();
                }

                if(dtb== DistanceToBackdrop.FAR && side==Side.RED && autopath==AutoPath.NO_SENSE){
                    redAllianceFar.start();
                    sleep(1000);
                    redAllianceFarThread.interrupt();
                }
                if(dtb== DistanceToBackdrop.CLOSE && side==Side.BLUE && autopath==AutoPath.NO_SENSE){
                    blueAllianceCloseThread.start();
                    sleep(1000);
                    blueAllianceFarThread.interrupt();
                }
                if(dtb== DistanceToBackdrop.CLOSE && side==Side.RED && autopath==AutoPath.NO_SENSE){
                    blueAllianceCloseThread.start();
                    sleep(1000);
                    redAllianceCloseThread.interrupt();
                }


            }
        }
    }

    private void outtake(){
        Bot.fourbar.outtake();
        Bot.slides.runTo(1);
        Bot.box.depositFirstPixel();
        Bot.slides.runTo(2);
        Bot.box.depositSecondPixel();
        Bot.resetOuttake();
    }

    private void dropPurplePixel(){

    }
}
