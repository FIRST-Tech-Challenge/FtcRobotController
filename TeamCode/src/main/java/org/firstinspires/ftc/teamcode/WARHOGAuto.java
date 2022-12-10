package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="WARHOGAuto", group="")
public class WARHOGAuto extends LinearOpMode {
    public WARHOGAuto() throws InterruptedException {}

    private StartPosColor startPosColor = StartPosColor.RED;
    private enum StartPosColor {
        RED, BLUE
    };
    private StartPosPosition startPosPosition = StartPosPosition.LEFT;
    private enum StartPosPosition{
        LEFT, RIGHT
    };

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    static final double FEET_PER_METER = 3.28084;

    int colorMod = 0;
    int posMod = 0;
    int cycles = 0;

    double speed = .85;

    //this stuff does not need to be changed
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    double tagsize = 0.166;

    //tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    int ID_TAG_OF_INTEREST = 18;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() throws InterruptedException {

        Drivetrain drivetrain = new Drivetrain(hardwareMap, telemetry);
        Intake intake = new Intake(hardwareMap, telemetry);
        Outtake outtake = new Outtake(hardwareMap, telemetry);

        intake.runArm(Intake.Height.UPRIGHT);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(864,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        //init loop
        while (!isStarted() && !isStopRequested()) {
            intake.runArm(Intake.Height.SIZING);
            //set up inputs - have previous so that you can check rising edge
            try {
                previousGamepad1.copy(currentGamepad1);
                previousGamepad2.copy(currentGamepad2);

                currentGamepad1.copy(gamepad1);
                currentGamepad2.copy(gamepad2);
            }
            catch (RobotCoreException e) {
                // Swallow the possible exception, it should not happen as
                // currentGamepad1/2 are being copied from valid Gamepads.
            }

            //set up initialization procedures
            if (currentGamepad1.b) {
                startPosColor = StartPosColor.RED;
            }
            if (currentGamepad1.x) {
                startPosColor = StartPosColor.BLUE;
            }
            if (currentGamepad1.dpad_left) {
                startPosPosition = StartPosPosition.LEFT;
            }
            if (currentGamepad1.dpad_right) {
                startPosPosition = StartPosPosition.RIGHT;
            }

            if(currentGamepad1.dpad_up && !previousGamepad1.dpad_up){
                cycles+=1;
            }
            if(currentGamepad1.dpad_down && !previousGamepad1.dpad_down){
                cycles-=1;
            }
            if(cycles>5){
                cycles=5;
            }
            if(cycles<-1){
                cycles=-1;
            }

            if(currentGamepad1.y && !previousGamepad1.y){
                speed+=.05;
            }
            if(currentGamepad1.a && !previousGamepad1.a){
                speed-=.05;
            }
            if(speed>1){
                speed=1;
            }
            if(speed<.4){
                speed=.4;
            }

            telemetry.addData("Color", startPosColor);
            telemetry.addData("Position", startPosPosition);
            telemetry.addData("Cycles", cycles);
            telemetry.addData("Speed", speed);
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            //detect apriltags
            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }


        // start command just came in

        //set modifier values
        switch (startPosColor){
            case RED:
                colorMod = 1;
                break;
            case BLUE:
                colorMod = -1;
                break;
        }
        switch (startPosPosition){
            case LEFT:
                posMod = -1;
                break;
            case RIGHT:
                posMod = 1;
                break;
        }

        outtake.closeClaw();

        // drive to pole and raise slide
        drivetrain.MoveForDis(60, speed);
        drivetrain.MoveForDis(-7.5, speed);
        if(cycles>-1) {
            drivetrain.rotateToPosition(-45 * posMod, speed - .25);
            drivetrain.MoveForDis(-1.5, speed);
            telemetry.addLine("just before slides");
            telemetry.update();
            outtake.setHeight(Outtake.Height.HIGH);
            telemetry.addLine("height added");
            telemetry.update();
            outtake.setHeight(1500);
            outtake.openClaw();
            outtake.setHeight(Outtake.Height.GROUND);

            // turn to cone stack
            //drivetrain.MoveForDis(1, speed);

            telemetry.addLine("Stage 1 complete");
        }
        else{
            cycles = 0;
        }

        for(int i = 0; i < cycles; i++) {
            //drivetrain.RotateForDegree(-45 * posMod, speed);
            drivetrain.rotateToPosition(-90 * posMod, speed-.35);
            intake.runArm(.2-.04*i);
            sleep(500);

            // move backward toward cone stack
            drivetrain.MoveForDis(-15, speed-.2);

            // take another cone
            intake.closeClaw();
            sleep(1000);
            intake.runArm(.4);
            sleep(1000);
            intake.runArm(Intake.Height.RETRACTED);

            // turn back
            drivetrain.MoveForDis(15, speed);
            intake.openClaw();
            //sleep(500);
            //drivetrain.RotateForDegree(45 * posMod, speed);
            drivetrain.rotateToPosition(-45 * posMod, speed-.25);
            intake.runArm(Intake.Height.UPRIGHT);
            outtake.closeClaw();
            drivetrain.MoveForDis(-1, 0.2);
            outtake.setHeight(Outtake.Height.HIGH);
            telemetry.addLine("height added");
            telemetry.update();
            outtake.setHeight(1500);
            outtake.openClaw();
            outtake.setHeight(Outtake.Height.GROUND);

            // turn to cone stack
            drivetrain.MoveForDis(1, speed);
        }
        telemetry.addLine("Stage 2 complete");
        telemetry.update();

        // park
        intake.runArm(Intake.Height.SIZING);
        //drivetrain.RotateForDegree(-45 * posMod, speed);
        drivetrain.rotateToPosition(-90 * posMod, speed-.25);
        if(tagOfInterest == null || tagOfInterest.id == MIDDLE){

        }else if((tagOfInterest.id-2)*posMod==1){

            drivetrain.MoveForDis(-24, speed);

        }else{
            drivetrain.MoveForDis(24, speed);
        }

        //drivetrain.RotateForDegree(90*posMod, speed);
        if(tagOfInterest==null || (tagOfInterest.id-2)*posMod==-1 || tagOfInterest.id==MIDDLE) {
            drivetrain.rotateToPosition(0, speed-.25);
            drivetrain.MoveForDis(-12, speed);
        }
        intake.runArm(Intake.Height.RETRACTED);
        telemetry.addLine("Stage 3 complete");
        telemetry.update();
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
