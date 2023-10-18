package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "CameraTest", group = "Linear autoMode")
public class AutoCameraTest extends LinearOpMode
{
    OpenCvWebcam webcam;
    org.openftc.easyopencv.OpenCvPipeline pipeline;
    org.openftc.easyopencv.OpenCvPipeline snapshotAnalysis = org.openftc.easyopencv.OpenCvPipeline.; // default

    @Override
    public void runOpMode()
    {
        
    }
}