package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.autonomous.TeamPropDetectionPipeline.TeamProp;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


/*
To Do:

1) TEST AUTOPATHS AND TELEOP!!!!
    - if splines do not work, switch to forward(), strafeRight(), and strafeLeft()
2) add more autopaths
3) odometry

 */

//*** Note: I created two pipelines, and assigned the camera to different pipeline at different times => this may create error


@Config
@Autonomous(name = "MainAutonomous")
public class MainAuto extends LinearOpMode{

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    Bot bot;
    double distanceFromObject;

    enum Side {
        RED, BLUE, NULL
    }
    enum DistanceToBackdrop{
        CLOSE, FAR, NULL
    }

    //different paths to follow depending on driver input before match
    enum AutoPath{
        MECHANICAL_FAILURE, NO_SENSE, OPTIMAL
    }

    private TeamProp teamPropLocation = TeamProp.NOTDETECTED;


    Side side = Side.NULL;
    DistanceToBackdrop dtb= DistanceToBackdrop.NULL;
    AutoPath autopath = AutoPath.OPTIMAL;


    double fx = 1078.03779;
    double fy = 1084.50988;
    double cx = 580.850545;
    double cy = 245.959325;

    // UNITS ARE METERS
    double tagSize = 0.032;
    OpenCvCamera camera;
    AprilTagsPipeline aprilTagsPipeline;



    @Override
    public void runOpMode() throws InterruptedException {
        bot = Bot.getInstance(this);
        GamepadEx gp1 = new GamepadEx(gamepad1);

        //different start positions depending on alliance and distance from backdrop
        Pose2d startPoseBlueFar = new Pose2d(-54, -36, 0);
        Pose2d startPoseBlueClose = new Pose2d(-54, 36, 0);
        Pose2d startPoseRedClose = new Pose2d(54, 36, 0);
        Pose2d startPoseRedFar = new Pose2d(54, -36, 0);


        //CAMERA STUFF =====================

        WebcamName camName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(camName);
        TeamPropDetectionPipeline teamPropDetectionPipeline = new TeamPropDetectionPipeline(telemetry);
        aprilTagsPipeline= new AprilTagsPipeline(tagSize, fx, fy, cx, cy);


        camera.setPipeline(teamPropDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });



        while (!isStarted() && !isStopRequested()) {
            gp1.readButtons();

            //Set distance, side, and auto type
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

            if(dtb == DistanceToBackdrop.FAR && side==Side.BLUE){
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


            //creating Trajectories/Paths
            TrajectorySequence blueAllianceFar = drive.trajectorySequenceBuilder(startPoseBlueFar)
                    //go to the position (-36,-36) from startPoseBlueFar => do not turn and keep constant acceleration
                    .splineTo(new Vector2d(-34,10), Math.toRadians(90))
                    //perform dropPurplePixel() at this moment
                    .addTemporalMarker(this::dropPurplePixel)
                    //go to position (-36,48) and turn 90 degrees
                    .splineTo(new Vector2d(42,38), Math.toRadians(0))
                    //perform moveBasedOnSpikeMark() 0.5 seconds before this moment
                    .UNSTABLE_addTemporalMarkerOffset(-0.5,this::moveBasedOnTeamProp)
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> bot.outtake(1, bot.distanceSensor,true))
                    .splineTo(new Vector2d(56,56), Math.toRadians(0))
                    .build();

            TrajectorySequence redAllianceFar= drive.trajectorySequenceBuilder(startPoseRedFar)
                    .splineTo(new Vector2d(36,-36), Math.toRadians(0))
                    .addTemporalMarker(this::dropPurplePixel)
                    .splineTo(new Vector2d(36,48), Math.toRadians(-90))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5,this::moveBasedOnTeamProp)
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> bot.outtake(1, Bot.distanceSensor,true))
                    .splineTo(new Vector2d(72,48), Math.toRadians(-90))
                    .build();

            TrajectorySequence blueAllianceClose = drive.trajectorySequenceBuilder(startPoseBlueClose)
                    .splineTo(new Vector2d(-54,20), Math.toRadians(0))
                    .splineTo(new Vector2d(-36,20), Math.toRadians(90))
                    .addTemporalMarker(this::dropPurplePixel)
                    .splineTo(new Vector2d(-36,48), Math.toRadians(90))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5,this::moveBasedOnTeamProp)
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> bot.outtake(1, Bot.distanceSensor,true))
                    .splineTo(new Vector2d(-72,48), Math.toRadians(90))
                    .build();

            TrajectorySequence redAllianceClose= drive.trajectorySequenceBuilder(startPoseRedClose)
                    .splineTo(new Vector2d(54,12), Math.toRadians(0))
                    .splineTo(new Vector2d(36,12), Math.toRadians(-90))
                    .addTemporalMarker(this::dropPurplePixel)
                    .splineTo(new Vector2d(36,48), Math.toRadians(-90))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5,this::moveBasedOnTeamProp)
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> bot.outtake(1, Bot.distanceSensor,true))
                    .splineTo(new Vector2d(72,48), Math.toRadians(90))
                    .build();

            TrajectorySequence redAllianceCloseNoSense = drive.trajectorySequenceBuilder(startPoseRedClose)
                    .splineTo(new Vector2d(54,12), Math.toRadians(0))
                    .splineTo(new Vector2d(36,12), Math.toRadians(-90))
                    .addTemporalMarker(this::dropPurplePixel)
                    .splineTo(new Vector2d(36,48), Math.toRadians(-90))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> bot.outtake(1, Bot.distanceSensor,true))
                    .splineTo(new Vector2d(72,48), Math.toRadians(90))
                    .build();

            TrajectorySequence blueAllianceCloseNoSense = drive.trajectorySequenceBuilder(startPoseBlueClose)
                    .splineTo(new Vector2d(-54,20), Math.toRadians(0))
                    .splineTo(new Vector2d(-36,20), Math.toRadians(90))
                    .addTemporalMarker(this::dropPurplePixel)
                    .splineTo(new Vector2d(-36,48), Math.toRadians(90))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> bot.outtake(1, Bot.distanceSensor,true))
                    .splineTo(new Vector2d(-72,48), Math.toRadians(90))
                    .build();

            TrajectorySequence blueAllianceFarNoSense = drive.trajectorySequenceBuilder(startPoseBlueFar)
                    .splineTo(new Vector2d(-34,10), Math.toRadians(90))
                    .addTemporalMarker(this::dropPurplePixel)
                    .splineTo(new Vector2d(42,38), Math.toRadians(0))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> bot.outtake(1, Bot.distanceSensor,true))
                    .splineTo(new Vector2d(56,56), Math.toRadians(0))
                    .build();

            TrajectorySequence redAllianceFarNoSense= drive.trajectorySequenceBuilder(startPoseRedFar)
                    .splineTo(new Vector2d(36,-36), Math.toRadians(0))
                    .addTemporalMarker(this::dropPurplePixel)
                    .splineTo(new Vector2d(36,48), Math.toRadians(-90))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> bot.outtake(1, Bot.distanceSensor,true))
                    .splineTo(new Vector2d(72,48), Math.toRadians(-90))
                    .build();



            waitForStart();
            if (!isStopRequested()) {

                if(dtb== DistanceToBackdrop.FAR && side==Side.BLUE && autopath==AutoPath.OPTIMAL){
                    //store the spikeMarkLocation based on input from team prop pipeline
                    findTeamPropLocation();
                    drive.followTrajectorySequence(blueAllianceFar);
                }

                else if(dtb== DistanceToBackdrop.FAR && side==Side.RED && autopath==AutoPath.OPTIMAL){
                    findTeamPropLocation();
                    drive.followTrajectorySequence(redAllianceFar);
                }
                else if(dtb== DistanceToBackdrop.CLOSE && side==Side.BLUE && autopath==AutoPath.OPTIMAL){
                    findTeamPropLocation();
                    drive.followTrajectorySequence(blueAllianceClose);
                }
                else if(dtb== DistanceToBackdrop.CLOSE && side==Side.RED && autopath==AutoPath.OPTIMAL){
                    findTeamPropLocation();
                    drive.followTrajectorySequence(redAllianceClose);
                }
                else if(dtb== DistanceToBackdrop.FAR && side==Side.BLUE && autopath==AutoPath.NO_SENSE){
                    drive.followTrajectorySequence(blueAllianceFarNoSense);
                }

                else if(dtb== DistanceToBackdrop.FAR && side==Side.RED && autopath==AutoPath.NO_SENSE){
                    drive.followTrajectorySequence(redAllianceFarNoSense);
                }
                else if(dtb== DistanceToBackdrop.CLOSE && side==Side.BLUE && autopath==AutoPath.NO_SENSE){
                    drive.followTrajectorySequence(blueAllianceCloseNoSense);
                }
                else if(dtb== DistanceToBackdrop.CLOSE && side==Side.RED && autopath==AutoPath.NO_SENSE){
                    drive.followTrajectorySequence(redAllianceCloseNoSense);
                }


            }
        }
    }

    private void dropPurplePixel(){

        if(teamPropLocation==TeamProp.ONLEFT){
            bot.turn(-0.25);
        }
        else if(teamPropLocation==TeamProp.ONRIGHT){
            bot.turn(0.25);
        }
        else{
            bot.forward();
        }
        Bot.noodles.reverseIntake();
    }

    private void findTeamPropLocation(){
        if(TeamPropDetectionPipeline.teamPropLocation== TeamProp.ONLEFT){
            teamPropLocation= TeamProp.ONLEFT;
        }

        else if(TeamPropDetectionPipeline.teamPropLocation== TeamProp.ONRIGHT){
            teamPropLocation= TeamProp.ONRIGHT;
        }
        else if(TeamPropDetectionPipeline.teamPropLocation== TeamProp.MIDDLE){
            teamPropLocation= TeamProp.MIDDLE;
        }
        else{
            teamPropLocation= TeamProp.NOTDETECTED;
        }
    }


    private void moveBasedOnTeamProp(){

        //switch to aprilTagsPipeline => looking for AprilTags
        camera.setPipeline(aprilTagsPipeline);
        int counter=0;

        //based on where team prop is, move to the corresponding position on the backdrop
        if(teamPropLocation== TeamProp.ONLEFT){

            //keep strafing left until robot detects AprilTag or if you have run loop over 5 times
            while(AprilTagsDetection.tagOfInterest.id!= 1 && counter<5){
                AprilTagsDetection.detectTag();
                bot.strafeLeft();
                counter++;
            }
        }

        else if(teamPropLocation== TeamProp.ONRIGHT){

            while(AprilTagsDetection.tagOfInterest.id!=3 && counter<5){
                AprilTagsDetection.detectTag();
                bot.strafeRight();
                counter++;
            }
        }

        else if(teamPropLocation == TeamProp.NOTDETECTED){
            telemetry.addData("Prop not detected, check pipeline", teamPropLocation);
        }
    }

}
