package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robot_utilities.VisionController;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous(name = "VisionTest")
public class VisionTest extends LinearOpMode {

    // Code for vision testing.
    VisionController visionController;
    OpenCvInternalCamera phoneCam;
    ElapsedTime elapsedTime;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        elapsedTime = new ElapsedTime();
        visionController = new VisionController(phoneCam);

        waitForStart();
        elapsedTime.reset();
        while(opModeIsActive()) {
            telemetry.addData("Position", visionController.getRingPosition());
//            telemetry.addData("Analysis", visionController.getAnalysis());
            telemetry.addData("Height", visionController.getHeight());
            telemetry.update();

            sleep(50);
        }


    }


}