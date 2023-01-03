package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.freightfrenzy.FreightFrenzyRobot;
import org.firstinspires.ftc.teamcode.freightfrenzy.RoadRunnerSubsystem;
import org.firstinspires.ftc.teamcode.opencvpipelines.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.robotbase.RobotEx;
import org.firstinspires.ftc.teamcode.myroadrunner.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@Autonomous(name = "Season: Freight Frenzy Autonomous")
public class FreightFrenzyAutonomous extends CommandOpMode {

    FreightFrenzyRobot robot;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    protected ElapsedTime runtime;
    protected SampleMecanumDrive drive;
    protected RoadRunnerSubsystem RR;
    protected Trajectory t1;
    protected Trajectory t2;
    protected Trajectory t3;

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
        GamepadEx driverOp = new GamepadEx(gamepad1);
        GamepadEx toolOp = new GamepadEx(gamepad2);

        robot = new FreightFrenzyRobot(hardwareMap, telemetry, driverOp, toolOp,
                RobotEx.OpModeType.AUTO);

        drive = new SampleMecanumDrive(hardwareMap);

        RR = new RoadRunnerSubsystem(drive);

        runtime = new ElapsedTime();

    }

    @Override
    public void waitForStart() {
        /////////////////////////////////// Rechgonizing the Tag ///////////////////////////////////
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
        //check for tag of interest == null command maybe wrongss

        RR.runT1();
        RR.runT2();

        while (runtime.seconds() < 20){

            RR.runT3();
            RR.runT4();

        }

        if (tagOfInterest.id == LEFT) RR.runT1();
        else if (tagOfInterest.id == MIDDLE) RR.runT2();
        else RR.runT3();


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