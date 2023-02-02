package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


import java.util.ArrayList;
import java.util.Random;

@Autonomous(name="PowerPlayAuton", group="Linear OpMode")
public class PowerPlayAuton extends LinearOpMode {

    private RobotManager robotManager;
    private ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //insert ID of sleeve
    int left = 1;
    int middle = 2;
    int right = 3;

    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                //camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
                camera.startStreaming(864,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        initSharedPreferences();
        robotManager = new RobotManager(hardwareMap, gamepad1, gamepad2, PowerPlayAuton.navigationPath,
                PowerPlayAuton.allianceColor, PowerPlayAuton.startingSide,
                PowerPlayAuton.movementMode, telemetry, elapsedTime);
        IMUPositioning.Initialize(this);
        robotManager.closeClaw();

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    //    if(tag.id == ID_TAG_OF_INTEREST)
                    if(tag.id == left || tag.id == middle || tag.id == right)
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

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        boolean moving = true;
        if(tagOfInterest == null || tagOfInterest.id == left)
        {
            System.out.println("Left");
            robotManager.navigation.setDriveMotorPowers(0, Navigation.MAX_STRAFE_POWER, 0, robotManager.robot, false);
            waitMilliseconds(100);

        }
        else if(tagOfInterest.id == middle) {
            System.out.println("middle");
            moving = false;
        }
        else if(tagOfInterest.id == right) {
            System.out.println("right");
            robotManager.navigation.setDriveMotorPowers(Math.PI, Navigation.MAX_STRAFE_POWER, 0, robotManager.robot, false);
        }

        if (moving) {
            waitMilliseconds(675);
            robotManager.navigation.stopMovement(robotManager.robot);
        }

        // Move forward
        robotManager.navigation.setDriveMotorPowers(-Math.PI / 2, Navigation.MAX_STRAFE_POWER, 0, robotManager.robot, false);
        waitMilliseconds(750);
        robotManager.navigation.stopMovement(robotManager.robot);

        while (opModeIsActive()) {}
    }


    private void waitMilliseconds(long ms) {
        double start_time = elapsedTime.time();
        while (elapsedTime.time() - start_time < ms) {}
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
    
    // ANDROID SHARED PREFERENCES
    // ==========================

    // Adapted from https://github.com/ver09934/twentytwenty/blob/ian-dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SkystoneAuton.java

    private static SharedPreferences sharedPrefs;

    private static int waitTime = 0;
    private static Navigation.MovementMode movementMode;
    private static RobotManager.StartingSide startingSide;
    private static RobotManager.AllianceColor allianceColor;
    private static ArrayList<Position> navigationPath;

    public void initSharedPreferences() {
        sharedPrefs = PreferenceManager.getDefaultSharedPreferences(this.hardwareMap.appContext);

        String movementMode = sharedPrefs.getString("movement_mode", "ERROR");
        String waitTime = sharedPrefs.getString("wait_time", "ERROR");
        String startingSide = sharedPrefs.getString("starting_side", "ERROR");
        String allianceColor = sharedPrefs.getString("alliance_color", "ERROR");
        String autonMode = sharedPrefs.getString("auton_type", "ERROR");

        telemetry.addData("Movement mode", movementMode);
        telemetry.addData("Wait time", waitTime);
        telemetry.addData("Auton mode", autonMode);
        telemetry.addData("Starting side", startingSide);
        telemetry.addData("Alliance color", allianceColor);

        System.out.println("Movement mode "+ movementMode);
        System.out.println("Wait time "+ waitTime);
        System.out.println("Auton mode "+ autonMode);
        System.out.println("Starting side "+ startingSide);
        System.out.println("Alliance color "+ allianceColor);


        switch (movementMode) {
            case "STRAFE":
                PowerPlayAuton.movementMode = Navigation.MovementMode.STRAFE;
                break;
            case "FORWARD_ONLY":
                PowerPlayAuton.movementMode = Navigation.MovementMode.FORWARD_ONLY;
                break;
        }

        switch (waitTime) {
            case "0_SECONDS":
                PowerPlayAuton.waitTime = 0;
                break;
            case "5_SECONDS":
                PowerPlayAuton.waitTime = 5;
                break;
            case "10_SECONDS":
                PowerPlayAuton.waitTime = 10;
                break;
            case "15_SECONDS":
                PowerPlayAuton.waitTime = 15;
                break;
            case "20_SECONDS":
                PowerPlayAuton.waitTime = 20;
                break;
        }

        switch(startingSide) {
            case "THEIR_COLOR":
                PowerPlayAuton.startingSide = RobotManager.StartingSide.THEIR_COLOR;
                break;

            case "OUR_COLOR":
                PowerPlayAuton.startingSide = RobotManager.StartingSide.OUR_COLOR;
                break;
        }

        if (allianceColor.equals("BLUE")) {
            PowerPlayAuton.allianceColor = RobotManager.AllianceColor.BLUE;
        }
        else if (allianceColor.equals("RED")) {
            PowerPlayAuton.allianceColor = RobotManager.AllianceColor.RED;
        }

        switch (autonMode) {
            case "SMALL":
                PowerPlayAuton.navigationPath = (ArrayList<Position>) AutonomousPaths.SMALL.clone();
                break;
            case "MEDIUM":
                PowerPlayAuton.navigationPath = (ArrayList<Position>) AutonomousPaths.MEDIUM.clone();
                break;
            case "LARGE":
                PowerPlayAuton.navigationPath = (ArrayList<Position>) AutonomousPaths.LARGE.clone();
                break;
            case "PARK_ONLY":
                PowerPlayAuton.navigationPath = (ArrayList<Position>) AutonomousPaths.PARK_ONLY.clone();
        }
    }
}
