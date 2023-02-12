package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.freightfrenzy.FreightFrenzyRobot;
import org.firstinspires.ftc.teamcode.powerplayAuto.RoadRunnerSubsystem;
import org.firstinspires.ftc.teamcode.opencvpipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.robotbase.GamepadExEx;
import org.firstinspires.ftc.teamcode.myroadrunner.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Autonomous(name = "Season: Freight Frenzy Autonomous")
public class FreightFrenzyAutonomous extends CommandOpMode {

    PowerPlayRobotV2Auto robot;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    protected ElapsedTime runtime;
    protected SampleMecanumDrive drive;
    protected RoadRunnerSubsystem RR;

    static final double FEET_PER_METER = 3.28084;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 484;
    int MIDDLE = 484;
    int RIGHT = 485;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void initialize() {
        GamepadExEx driverOp = new GamepadExEx(gamepad1);
        GamepadExEx toolOp = new GamepadExEx(gamepad2);

        robot = new PowerPlayRobotV2Auto(hardwareMap, telemetry, driverOp, toolOp);

        drive = new SampleMecanumDrive(hardwareMap);

        RR = new RoadRunnerSubsystem(drive);

        runtime = new ElapsedTime();

    }

    @Override
    public void waitForStart() {
        /////////////////////////////////// Recognizing the Tag ///////////////////////////////////
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = robot.camera.getPipeline().getLatestDetections();
            if (currentDetections.size() != 0) {
                boolean tagFound = false;
                for (AprilTagDetection tag : currentDetections) {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
                ////////////////////////////// Live Tag State //////////////////////////////////////
                if(tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    if(tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        tagToTelemetry(tagOfInterest);
                    }
                }
            } else {
                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    tagToTelemetry(tagOfInterest);
                }
            }
            telemetry.update();
            sleep(20);
        }
    }

    @Override
    public void run() {
        super.run();
        // TODO: Make telemetry subsystem/command and remove this function
        robot.telemetryUpdate();
        robot.dashboardTelemetryUpdate();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine(
                    "Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
        }
        telemetry.update();

        ///////////////////////////////// Running the Trajectories /////////////////////////////////
//        if (isStopRequested()) return;

        /* Actually do something useful */
        //check for tag of interest == null command maybe wrong

        RR.runHM();
        RR.runMR();

        while (runtime.seconds() < 20){

            RR.runRG();
            RR.runGR();

        }

        RR.runTOMID();

        if (tagOfInterest.id == LEFT) RR.runP1();
        else if (tagOfInterest.id == MIDDLE) RR.runP2();
        else RR.runP3();


        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
        }
        reset();
    }

    /////////////////////////// Pushing everything in the Telemetry ////////////////////////////
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

//    @Override
//    public void run() {
//        super.run();
//        // TODO: Make telemetry subsystem/command and remove this function
//        robot.telemetryUpdate();
//        robot.dashboardTelemetryUpdate();
//    }
}