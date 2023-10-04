package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/*

To Do:

1) change startPose for each alliance/position
2) drop purple pixel method
3) incorporate AprilTags
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

    @Override
    public void runOpMode() throws InterruptedException {
        bot = Bot.getInstance(this);
        GamepadEx gp1 = new GamepadEx(gamepad1);

        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);


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


            TrajectorySequence blueAllianceFar = drive.trajectorySequenceBuilder(startPose)
                    .splineTo(new Vector2d(-36,-36), Math.toRadians(0))
                    .addTemporalMarker(this::dropPurplePixel)
                    .splineTo(new Vector2d(-36,48), Math.toRadians(90))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, this::outtake)
                    .splineTo(new Vector2d(-72,48), Math.toRadians(90))
                    .build();

            TrajectorySequence redAllianceFar= drive.trajectorySequenceBuilder(startPose)
                    .splineTo(new Vector2d(36,-36), Math.toRadians(0))
                    .addTemporalMarker(this::dropPurplePixel)
                    .splineTo(new Vector2d(36,48), Math.toRadians(-90))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, this::outtake)
                    .splineTo(new Vector2d(72,48), Math.toRadians(-90))
                    .build();

            TrajectorySequence blueAllianceClose = drive.trajectorySequenceBuilder(startPose)
                    .splineTo(new Vector2d(-54,20), Math.toRadians(0))
                    .splineTo(new Vector2d(-36,20), Math.toRadians(90))
                    .addTemporalMarker(this::dropPurplePixel)
                    .splineTo(new Vector2d(-36,48), Math.toRadians(90))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, this::outtake)
                    .splineTo(new Vector2d(-72,48), Math.toRadians(90))
                    .build();

            TrajectorySequence redAllianceClose= drive.trajectorySequenceBuilder(startPose)
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
                    blueAllianceFarThread.interrupt();
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

    private void readAprilTag(){
        //read april tag and perform actions accordingly
        //then incorporate into trajectory
    }
}
