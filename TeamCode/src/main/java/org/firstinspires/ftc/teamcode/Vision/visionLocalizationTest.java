package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.Locale;

@TeleOp
@Config
public class visionLocalizationTest extends LinearOpMode {
    int Portal_1_View_ID;
    ArrayList<AprilTagDetection> detections;
    Pose2d robotPos;
    double fx = 883.846030309f;
    double fy = 883.846030309f;
    double cx = 416.860113959f;
    double cy = 295.808977317f;
    public static double ox = 2.784;
    public static double oz = 8;
    static final double FEET_PER_METER = 3.28084;

    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        Portal_1_View_ID = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VisionLocalizer visionLocalizer = new VisionLocalizer(fx, fy, cx, cy, hardwareMap, "Webcam 1", Portal_1_View_ID, ox, oz);
        visionLocalizer.startStreaming();

        waitForStart();
        telemetry.setMsTransmissionInterval(50);
        while (opModeIsActive() && !isStopRequested()) {
            drive.update();


            robotPos = visionLocalizer.getGlobalPos();
            if ((robotPos == null)) {
                robotPos = drive.getPoseEstimate();
                telemetry.addLine(String.format(Locale.US, "Robot x: %.2f inches", robotPos.getX()));
                telemetry.addLine(String.format(Locale.US, "Robot y: %.2f inches", robotPos.getY()));
                telemetry.addLine(String.format(Locale.US, "Robot heading: %.2f degrees\n", Math.toDegrees(robotPos.getHeading())));
            } else {


                drive.setPoseEstimate(robotPos);
                telemetry.addLine(String.format(Locale.US, "Robot x: %.2f inches", robotPos.getX()));
                telemetry.addLine(String.format(Locale.US, "Robot y: %.2f inches", robotPos.getY()));
                telemetry.addLine(String.format(Locale.US, "Robot heading: %.2f degrees\n", Math.toDegrees(robotPos.getHeading())));
            }


            telemetry.update();
        }
        visionLocalizer.stopStreaming();
    }
}
