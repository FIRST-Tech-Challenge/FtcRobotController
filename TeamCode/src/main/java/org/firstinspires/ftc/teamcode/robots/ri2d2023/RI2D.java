package org.firstinspires.ftc.teamcode.robots.ri2d2023;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robots.taubot.util.StickyGamepad;
import org.firstinspires.ftc.teamcode.robots.taubot.vision.pipeline.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config ("RI2D2023GameVariables")
@TeleOp(name="RI2D OpMode", group="Challenge")
public class RI2D extends OpMode {
    private StickyGamepad stickyGamepad1, stickyGamepad2;
    //autonomous variables
    public boolean auton = false; // controls if auton will run set to true to run with auton
    public static  boolean testing = false;// turns off normal robot motion
    public static boolean red = true; // team boolean variable red true is red team
    //miscellaneous variables
    public static boolean calibrateOn = true;// turns off automatic elevator calibration
    private boolean calibrate = false;
    public static float DEADZONE = .1f;
    //vision variables
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;
    int tagDetected = 0;
    // UNITS ARE PIXELS
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    // UNITS ARE METERS
    double tagsize = 0.045; //tag size on iron reign signal sleeve
    int ID_TAG_OF_INTEREST = 1; // Tag ID 1 from the 36h11 family
    int tagCount = 0;
    boolean tagFound = false;
    AprilTagDetection tagOfInterest = null;
    //Robot variable storage system
    Robot robot;
    //autonomous program
    Autonomous autonomous;
    //Cone stack for auton
    @Override
    public void init() {
        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);
        robot = new Robot(telemetry, hardwareMap);
        autonomous = new Autonomous(robot);
        telemetry.addData("Status", "Initializing " + this.getClass() + "...");
        telemetry.update();
        robot.init();
        //visionInit();
    }

    @Override
    public void init_loop() {
        //aprilTagInitLoop();
        telemetry.update();
    }
    @Override
    public void loop() {
        stickyGamepad1.update();
        stickyGamepad2.update();
        if(auton) {
//            autonInit(tagDetected);
        }
        telemetryOutput();
        if(!testing){
            telemetry.addData("should auton be running? \t", autonomous.hasBehaviors());
            /*if(autonomous.hasBehaviors()) {
                autonomous.runBehaviors();
            }
            else {*/
                robot.driveTrain.mechanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
//                robot.driveTrain.tankDrive(gamepad1.left_stick_y, gamepad1.right_stick_y);
                if(gamepad1.left_bumper)
                    robot.intake.clawOpen();
                if(gamepad1.right_bumper)
                    robot.intake.clawClose();
                if(gamepad1.left_trigger > DEADZONE)
                    robot.intake.moveClawArm(gamepad1.left_trigger);
                if(gamepad1.right_trigger > DEADZONE)
                    robot.intake.moveClawArm(-gamepad1.right_trigger);
                if(gamepad1.a)
                    robot.outtake.moveSlide(1);
                if(gamepad1.b)
                    robot.outtake.moveSlide(-1);
                if(stickyGamepad1.y)
                    robot.outtake.flip();
                if (gamepad1.dpad_down) {
                    calibrate = false;
                }
                if(gamepad1.dpad_up) {
                    if (robot.driveTrain.robotSpeed == 1)
                        robot.driveTrain.robotSpeed = .5;
                    else
                        robot.driveTrain.robotSpeed = 1;
                }
            //}
        }
    }
    public void telemetryOutput() {
        telemetry.addData("is in auton \t", auton);
        telemetry.addData("tag value: \t", tagDetected);
        robot.driveTrain.telemetryOutput();
        robot.intake.telemetryOutput();
    }
    public void autonInit(int tagValue) {
        telemetry.addData("tag", tagValue);
    }
    // vision setup
    public void visionInit(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);
    }
    public void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("\nTag Count ID=%d", tagCount));
    }

    public void aprilTagInitLoop() {
        tagCount = 0;

        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if (currentDetections.size() != 0) {
            tagFound = false;

            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == 1 || tag.id == 2 || tag.id == 3) {
                    tagOfInterest = tag;
                    tagFound = true;
                    tagCount++;
                    break;
                }
            }

            if (tagFound) {
                telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                tagDetected = tagOfInterest.id;
                tagToTelemetry(tagOfInterest);

            } else {
                tagCount = 0;
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen a tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }

        } else {
            tagCount = 0;
            telemetry.addLine("Don't see tag of interest :(");

            if (tagOfInterest == null) {
                telemetry.addLine("(A tag has never been seen)");
            } else {
                telemetry.addLine("\nBut we HAVE seen a tag before; last seen at:");
                tagToTelemetry(tagOfInterest);
            }

        }

    }

    public void autonVisionTelemetry() {
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            auton = true;
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("Tag was not detected, only dropping off the cone");
            telemetry.update();
        }
    }
}