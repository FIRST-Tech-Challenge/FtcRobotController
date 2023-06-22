package org.firstinspires.ftc.teamcode.opModes.team1.auto;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.components.HDriveWrapper;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.motors.TrapezoidalProfile;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.motors.TrapezoidalProfileByTime;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TeamColour;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TelemetryContainer;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

public class GenericAuto
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;


    // Lens intrinsics
    // UNITS ARE PIXELS
    double fx = 678.154;
    double fy = 678.17;
    double cx = 400.898;
    double cy = 300.79;

    // UNITS ARE METERS
    double tagsize = 0.03;

    // Tag IDs 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    private ElapsedTime runtime = new ElapsedTime();
    private HDriveWrapper drive;
    private IMU imu;

    AprilTagDetection tagOfInterest = null;

    public void run(TeamColour teamColour, LinearOpMode opMode)
    {
        // Set up auto driving
        imu = HardwareMapContainer.getMap().get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR));
        imu.initialize(parameters);

        imu.resetYaw();

        // Get the third motor as a spinner motor
        drive = new HDriveWrapper(new HDrive(
                HardwareMapContainer.motor0,
                HardwareMapContainer.motor1,
                HardwareMapContainer.motor3,
                0,
                Math.PI,
                Math.PI/2
        ), imu);

        // Set up CV
        HardwareMap hardwareMap = HardwareMapContainer.getMap();
        MultipleTelemetry telemetry = TelemetryContainer.getTelemetry();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,600, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        runtime.reset();

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while ((!opMode.isStarted() && !opMode.isStopRequested()) || (tagOfInterest == null && runtime.seconds() < 15.0 && !opMode.isStopRequested()))
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

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
                    tagToTelemetry(tagOfInterest, telemetry);
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
                        tagToTelemetry(tagOfInterest, telemetry);
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
                    tagToTelemetry(tagOfInterest, telemetry);
                }

            }

            telemetry.update();
            opMode.sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest, telemetry);
            telemetry.update();
        }
        else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        final double AWAYFROMWALL_TIME = 1.0; // To travel away from the wall at the start so don't scrape on it
        final double ONETILE_TIME = 2.0; // To travel 2'
        final double INTOZONE_TIME = 3.2; // To travel ~3'

        /* Drive to zone based on tag seen */
        // Start facing right when looking at wall from pitch, with left side touching wall - allowed in Traditional Game Manual 2, Top of Page 15
        // Move off wall
        driveByTime(AWAYFROMWALL_TIME, true, true, opMode, telemetry);
        // Move to correct zone by wall
        if (tagOfInterest != null) {
            if (tagOfInterest.id == 1) {
                driveByTime(ONETILE_TIME, false, false, opMode, telemetry);
            } else if (tagOfInterest.id == 3) {
                driveByTime(ONETILE_TIME, false, true, opMode, telemetry);
            }
        }
        // Else 2; don't move to different zone
        driveByTime(INTOZONE_TIME, true, true, opMode, telemetry);
    }

    void tagToTelemetry(AprilTagDetection detection, MultipleTelemetry telemetry)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f metres", detection.pose.x));
        telemetry.addLine(String.format("Translation Y: %.2f metres", detection.pose.y));
        telemetry.addLine(String.format("Translation Z: %.2f metres", detection.pose.z));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    void driveByTime(double timeSeconds, boolean moveSideways, boolean moveBackwards, LinearOpMode opMode, MultipleTelemetry telemetry) {
        runtime.reset();
        int initialCounts = HDriveWrapper.getTotalEncoderCounts(HardwareMapContainer.motor0, HardwareMapContainer.motor1, HardwareMapContainer.motor3);

        TrapezoidalProfileByTime profile = new TrapezoidalProfileByTime(timeSeconds, new TrapezoidalProfile(0.7, 1));

        double speedMultiplier = moveBackwards ? -1 : 1;

        while(!opMode.isStopRequested() && opMode.opModeIsActive() && (runtime.seconds() < timeSeconds)) {
            telemetry.addData("Driving Time", runtime.seconds());
            telemetry.addData("Driving Counts", HDriveWrapper.getTotalEncoderCounts(HardwareMapContainer.motor0, HardwareMapContainer.motor1, HardwareMapContainer.motor3));
            telemetry.addData("Driving Distance (inches)", DriveConstants.encoderTicksToInches((double)(HDriveWrapper.getTotalEncoderCounts(HardwareMapContainer.motor0, HardwareMapContainer.motor1, HardwareMapContainer.motor3) - initialCounts) / 2)); // /2 as 2 motors
            if(moveSideways) { // Relative to original heading
                drive.fieldOrientedDriveAbsoluteRotation(0, speedMultiplier * profile.predict(runtime.seconds())); // So /``\ in velocity (limit acceleration)
            } else {
                drive.fieldOrientedDriveAbsoluteRotation(speedMultiplier * profile.predict(runtime.seconds()), 0); // So /``\ in velocity (limit acceleration)
            }
            telemetry.update();
            opMode.sleep(50);
        }
        HardwareMapContainer.motor0.set(0);
        HardwareMapContainer.motor1.set(0);
        HardwareMapContainer.motor3.set(0);
    }
}