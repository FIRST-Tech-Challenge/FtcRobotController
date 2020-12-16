package org.firstinspires.ftc.teamcode.robot_common;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvInternalCamera;
//import org.darbots.darbotsftclib.main.java.*;
import org.darbots.darbotsftclib.main.java.org.openftc.easyopencv.OpenCvCamera;
import org.darbots.darbotsftclib.main.java.org.openftc.easyopencv.OpenCvInternalCamera;
import org.darbots.darbotsftclib.main.java.org.openftc.easyopencv.OpenCvCameraFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Autonomous(name= "TeamCode Autonomous")
public class Robot4100Common_Template extends LinearOpMode {
    private OpenCvCamera phoneCam;

    @Override
    public void runOpMode (){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamname = hardwareMap.get(WebcamName.class, "Webcam 1");
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(webcamname, cameraMonitorViewId);
        phoneCam.openCameraDevice();
    }
}

