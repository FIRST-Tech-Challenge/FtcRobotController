package org.firstinspires.ftc.teamcode.Tests.TeleopTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.DetectMarker.DetectMarker;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.DetectMarker.DetectMarkerThread;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.DetectMarker.MarkerLocation;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.io.IOException;

@TeleOp(name="Detect Marker Test")
public class DetectMarkerTest extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode()
    {
        Robot robot;
        robot = new Robot(this, timer);
        waitForStart();

        while (opModeIsActive()) {
            OpenCvInternalCamera robotCamera;
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            robotCamera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

            DetectMarker detectMarkerRunnable = new DetectMarker(robot, robotCamera);
            MarkerLocation finalMarkerLocation = detectMarkerRunnable.DetectMarkerRun();
            telemetry.addData("Marker Location: ", "found");
            telemetry.update();
        }
    }
}
