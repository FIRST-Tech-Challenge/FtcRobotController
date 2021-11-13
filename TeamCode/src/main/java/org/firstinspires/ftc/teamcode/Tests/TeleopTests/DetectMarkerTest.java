package org.firstinspires.ftc.teamcode.Tests.TeleopTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.DetectMarker.DetectMarker;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.DetectMarker.MarkerLocation;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;

/**
 * Tests the detect marker capability of the robot
 */
@TeleOp(name = "Detect Marker Test")
public class DetectMarkerTest extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        Robot robot;
        robot = new Robot(this, timer, true);
        waitForStart();

        while (opModeIsActive()) {
            OpenCvInternalCamera robotCamera;
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            robotCamera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

            DetectMarker detectMarkerRunnable = new DetectMarker(hardwareMap, robotCamera, robot.getQuickTelemetry());
            MarkerLocation finalMarkerLocation = detectMarkerRunnable.DetectMarkerRun();
            telemetry.addData("Marker Location: ", "found");
            telemetry.update();
        }
    }
}
